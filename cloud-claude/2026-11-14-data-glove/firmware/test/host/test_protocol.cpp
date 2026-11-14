// PC-side unit tests for the glove wire protocol (no ESP32 needed):
//   crc16.h (CRC-16/CCITT-FALSE check value + CRCs from the reveng catalogue), cobs.h (edge cases
//   + random round trips), protocol.h (layout, seal/validate, corruption detection, SYNC bits),
//   spsc_ring.h (order, overflow drops, two-thread stress), glove_config.h (SensorMap invariants
//   are static_asserts; this prints the table).
// It also writes a synthetic USB stream (COBS frames, as the dongle sends) to the path given as
// argv[1], which tools/glove_reader.py --file decodes as a cross-language check.
//
// Run from firmware/:
//   g++ -std=c++17 -O2 -Wall -Wextra -pthread -Iinclude test/host/test_protocol.cpp -o .pio/test_protocol && .pio/test_protocol .pio/host_frames.bin
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <thread>
#include <vector>

#include "cobs.h"
#include "crc16.h"
#include "glove_config.h"
#include "protocol.h"
#include "spsc_ring.h"

namespace {
int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failures;                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
    }                                                                        \
  } while (0)

void test_crc() {
  const char* s = "123456789";
  CHECK(tg::crc16_ccitt(s, 9) == 0x29B1);   // CRC-16/CCITT-FALSE check value
  CHECK(tg::crc16_ccitt("", 0) == 0xFFFF);
  const uint8_t a[] = {'A'};
  CHECK(tg::crc16_ccitt(a, 1) == 0xB915);
  // Incremental == one-shot.
  uint16_t c = tg::crc16_ccitt_update(tg::kCrc16Init, reinterpret_cast<const uint8_t*>(s), 4);
  c = tg::crc16_ccitt_update(c, reinterpret_cast<const uint8_t*>(s) + 4, 5);
  CHECK(c == 0x29B1);
}

std::vector<uint8_t> cobs_rt(const std::vector<uint8_t>& in, size_t* enc_len = nullptr) {
  std::vector<uint8_t> enc(tg::cobs_max_encoded(in.size()));
  const size_t n = tg::cobs_encode(in.data(), in.size(), enc.data());
  if (enc_len) *enc_len = n;
  for (size_t i = 0; i < n; ++i)
    if (enc[i] == 0) return {0xEE};   // encoded data must not contain zeros
  std::vector<uint8_t> dec(in.size() + 8);
  const size_t m = tg::cobs_decode(enc.data(), n, dec.data(), dec.size());
  dec.resize(m);
  return dec;
}

void test_cobs() {
  size_t n;
  CHECK(cobs_rt({0x00}, &n) == std::vector<uint8_t>({0x00}) && n == 2);
  CHECK(cobs_rt({0x00, 0x00}) == std::vector<uint8_t>({0x00, 0x00}));
  CHECK(cobs_rt({0x11, 0x22, 0x00, 0x33}) == std::vector<uint8_t>({0x11, 0x22, 0x00, 0x33}));
  std::vector<uint8_t> v254(254, 0x01), v255(255, 0x02), v600(600, 0x03);
  CHECK(cobs_rt(v254, &n) == v254 && n == 255);
  CHECK(cobs_rt(v255) == v255);
  CHECK(cobs_rt(v600) == v600);
  // Known vector (Wikipedia): 11 22 00 33 -> 03 11 22 02 33
  uint8_t enc[8];
  const uint8_t in[] = {0x11, 0x22, 0x00, 0x33};
  CHECK(tg::cobs_encode(in, 4, enc) == 5);
  const uint8_t expect[] = {0x03, 0x11, 0x22, 0x02, 0x33};
  CHECK(std::memcmp(enc, expect, 5) == 0);
  // Malformed: code byte pointing past the end, embedded zero.
  uint8_t out[16];
  const uint8_t bad1[] = {0x05, 0x11};
  CHECK(tg::cobs_decode(bad1, 2, out, sizeof(out)) == 0);
  const uint8_t bad2[] = {0x03, 0x00, 0x11};
  CHECK(tg::cobs_decode(bad2, 3, out, sizeof(out)) == 0);
  // Random round trips, including all-zero and zero-free data.
  std::mt19937 rng(1234);
  for (int t = 0; t < 2000; ++t) {
    std::vector<uint8_t> v(rng() % 400);
    const int mode = t % 3;
    for (auto& b : v) b = mode == 0 ? static_cast<uint8_t>(rng()) : mode == 1 ? 0 : static_cast<uint8_t>(1 + rng() % 255);
    if (cobs_rt(v) != v) {
      CHECK(false);
      break;
    }
  }
  CHECK(true);
}

tg::DataPacket make_frame(uint32_t seq, tg::Side side) {
  tg::DataPacket p;
  std::memset(&p, 0, sizeof(p));
  tg::fill_header(p.hdr, tg::PacketType::Data, side, 20);
  p.seq = seq;
  p.t_us = 1000000ull + 5000ull * seq;
  p.sweep_us = 3900;
  p.sensor_ok = (1u << kSensorCount) - 1;
  p.status = tg::kStatusCharging;
  p.batt_soc = 87;
  p.batt_mv = 3987;
  for (int i = 0; i < tg::kNumMag; ++i)
    for (int a = 0; a < 3; ++a) p.mag[i][a] = static_cast<int16_t>((i + 1) * 100 * (a == 1 ? -1 : 1) + static_cast<int>(seq % 7));
  for (int i = 0; i < tg::kNumImu; ++i) {
    p.imu[i][2] = 8192;                 // 1 g on z
    p.imu[i][3] = static_cast<int16_t>(328 * (i + 1));   // 10 / 20 dps
    p.imu_temp[i] = static_cast<int16_t>(132.48f * 8.0f);  // 33 °C
  }
  tg::seal(p);
  return p;
}

void test_packets() {
  std::printf("sizeof: Data %zu, Event %zu, Hello %zu, HelloAck %zu, Command %zu, CommandAck %zu, UsbHeader %zu\n",
              sizeof(tg::DataPacket), sizeof(tg::EventPacket), sizeof(tg::HelloPacket), sizeof(tg::HelloAckPacket),
              sizeof(tg::CommandPacket), sizeof(tg::CommandAckPacket), sizeof(tg::UsbHeader));
  std::printf("DataPacket offsets: seq %zu t_us %zu sweep_us %zu sensor_ok %zu mag_saturated %zu status %zu "
              "batt_soc %zu batt_mv %zu ring_drops %zu tx_fail %zu mag %zu imu %zu imu_temp %zu crc %zu\n",
              offsetof(tg::DataPacket, seq), offsetof(tg::DataPacket, t_us), offsetof(tg::DataPacket, sweep_us),
              offsetof(tg::DataPacket, sensor_ok), offsetof(tg::DataPacket, mag_saturated),
              offsetof(tg::DataPacket, status), offsetof(tg::DataPacket, batt_soc), offsetof(tg::DataPacket, batt_mv),
              offsetof(tg::DataPacket, ring_drops), offsetof(tg::DataPacket, tx_fail), offsetof(tg::DataPacket, mag),
              offsetof(tg::DataPacket, imu), offsetof(tg::DataPacket, imu_temp), offsetof(tg::DataPacket, crc));
  CHECK(sizeof(tg::DataPacket) < 250);
  CHECK(offsetof(tg::DataPacket, crc) == sizeof(tg::DataPacket) - 2);

  tg::DataPacket p = make_frame(42, tg::Side::Left);
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&p);
  CHECK(raw[0] == 'T' && raw[1] == 'G');   // magic on the wire
  CHECK(tg::validate(raw, sizeof(p)) == static_cast<uint8_t>(tg::PacketType::Data));
  CHECK(tg::validate(raw, sizeof(p) - 1) == 0);   // wrong length
  // Every single-bit flip is caught (CRC-16 detects all 1-bit errors).
  int missed = 0;
  std::vector<uint8_t> buf(raw, raw + sizeof(p));
  for (size_t i = 0; i < buf.size(); ++i)
    for (int b = 0; b < 8; ++b) {
      buf[i] ^= static_cast<uint8_t>(1u << b);
      if (tg::validate(buf.data(), buf.size()) != 0) ++missed;
      buf[i] ^= static_cast<uint8_t>(1u << b);
    }
  CHECK(missed == 0);
  // Little-endian field placement.
  uint32_t seq;
  std::memcpy(&seq, raw + offsetof(tg::DataPacket, seq), 4);
  CHECK(seq == 42);
  CHECK(raw[offsetof(tg::DataPacket, batt_soc)] == 87);

  tg::EventPacket e;
  tg::fill_header(e.hdr, tg::PacketType::Event, tg::Side::Right);
  e.seq = 7;
  e.t_us = 123456789;
  e.event = static_cast<uint8_t>(tg::EventType::SyncEdge);
  e.arg = 0xA;
  e.arg2 = 0;
  tg::seal(e);
  CHECK(tg::validate(reinterpret_cast<uint8_t*>(&e), sizeof(e)) == static_cast<uint8_t>(tg::PacketType::Event));
  e.hdr.version = 2;
  CHECK(tg::validate(reinterpret_cast<uint8_t*>(&e), sizeof(e)) == 0);   // wrong protocol version

  // SYNC blink pattern: start bit then code MSB first. 0xA = 1010 -> 1 1 0 1 0.
  const bool expect[] = {true, true, false, true, false};
  for (int s = 0; s < tg::kSyncBits; ++s) CHECK(tg::sync_slot_on(0xA, s) == expect[s]);
  CHECK(!tg::sync_slot_on(0xF, 5));
}

void test_ring() {
  tg::SpscRing<tg::DataPacket, 4> r;
  tg::DataPacket out;
  CHECK(!r.pop(out));
  for (uint32_t i = 0; i < 6; ++i) r.push(make_frame(i, tg::Side::Left));
  CHECK(r.size() == 4 && r.drops() == 2);   // newest two dropped
  for (uint32_t i = 0; i < 4; ++i) {
    CHECK(r.pop(out) && out.seq == i);
  }
  CHECK(!r.pop(out));

  // Two-thread stress: consumer sees a strictly increasing seq; pushes = pops + drops.
  static tg::SpscRing<tg::DataPacket, 32> big;
  constexpr uint32_t kN = 200000;
  uint32_t popped = 0, bad = 0;
  std::thread prod([&] {
    for (uint32_t i = 1; i <= kN; ++i) big.push(make_frame(i, tg::Side::Right));
  });
  std::thread cons([&] {
    uint32_t last = 0;
    tg::DataPacket f;
    for (;;) {
      if (big.pop(f)) {
        if (f.seq <= last || tg::validate(reinterpret_cast<uint8_t*>(&f), sizeof(f)) == 0) ++bad;
        last = f.seq;
        ++popped;
        if (f.seq == kN) break;
      } else if (popped + big.drops() >= kN) {
        break;
      }
    }
  });
  prod.join();
  cons.join();
  tg::DataPacket f;
  while (big.pop(f)) ++popped;
  CHECK(bad == 0);
  CHECK(popped + big.drops() == kN);
  std::printf("ring stress: %u popped, %u dropped\n", popped, big.drops());
}

void print_sensor_map() {
  std::printf("SensorMap (%d sensors):\n", kSensorCount);
  for (int i = 0; i < kSensorCount; ++i)
    std::printf("  bit %2d  %-11s ch%u 0x%02X %s slot %u\n", i, kSensorMap[i].name, kSensorMap[i].channel,
                kSensorMap[i].addr, kSensorMap[i].type == SensorType::Tmag5273 ? "TMAG5273" : "ICM42688",
                kSensorMap[i].slot);
}

// Writes a synthetic dongle USB stream: left glove 200 Hz with a +50 ppm clock and a gap,
// right glove at -30 ppm, one SYNC event, one status frame and one garbage frame.
void write_stream(const char* path) {
  FILE* f = std::fopen(path, "wb");
  if (!f) {
    CHECK(false);
    return;
  }
  auto emit = [&](tg::UsbKind kind, uint8_t side, uint64_t rx, int8_t rssi, const void* payload, size_t len) {
    uint8_t raw[tg::kUsbMaxFrame];
    tg::UsbHeader h{static_cast<uint8_t>(kind), side, rx, rssi, static_cast<uint8_t>(len)};
    std::memcpy(raw, &h, sizeof(h));
    std::memcpy(raw + sizeof(h), payload, len);
    uint8_t enc[tg::cobs_max_encoded(tg::kUsbMaxFrame) + 1];
    size_t n = tg::cobs_encode(raw, sizeof(h) + len, enc);
    enc[n++] = 0;
    std::fwrite(enc, 1, n, f);
  };
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> lat(800, 3000);   // radio + USB latency, µs
  const double ppm[2] = {50.0, -30.0};
  const int64_t offs[2] = {-400000, 2500000};          // glove clock - dongle clock, µs
  for (uint32_t seq = 1; seq <= 2000; ++seq) {          // 10 s
    for (int g = 0; g < 2; ++g) {
      if (g == 0 && seq >= 500 && seq < 510) continue;  // 10 lost frames on the left glove
      tg::DataPacket p = make_frame(seq, g == 0 ? tg::Side::Left : tg::Side::Right);
      const double t_dongle = 1.0e6 + seq * 5000.0;
      p.t_us = static_cast<uint64_t>(t_dongle * (1.0 + ppm[g] * 1e-6) + offs[g]);
      tg::seal(p);
      emit(tg::UsbKind::GlovePacket, p.hdr.side, static_cast<uint64_t>(t_dongle + lat(rng)), -55, &p, sizeof(p));
    }
    if (seq == 1000) {
      tg::EventPacket e;
      tg::fill_header(e.hdr, tg::PacketType::Event, tg::Side::Left);
      e.seq = seq;
      e.t_us = 6000000;
      e.event = static_cast<uint8_t>(tg::EventType::SyncEdge);
      e.arg = 0x9;
      e.arg2 = 0;
      tg::seal(e);
      emit(tg::UsbKind::GlovePacket, 'L', 6001000, -55, &e, sizeof(e));
      const uint8_t junk[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
      emit(tg::UsbKind::GlovePacket, 'R', 6001500, -60, junk, sizeof(junk));   // bad CRC
      tg::DongleStatus st{};
      st.uptime_ms = 6000;
      emit(tg::UsbKind::DongleStatus, '?', 6002000, 0, &st, sizeof(st));
      const char txt[] = "hello from the test";
      emit(tg::UsbKind::Text, '?', 6002500, 0, txt, sizeof(txt) - 1);
    }
  }
  std::fclose(f);
  std::printf("wrote synthetic stream: %s (L: +50 ppm, offset -400000 us, 10 drops; R: -30 ppm, offset +2500000 us)\n",
              path);
}
}  // namespace

int main(int argc, char** argv) {
  test_crc();
  test_cobs();
  test_packets();
  test_ring();
  print_sensor_map();
  if (argc > 1) write_stream(argv[1]);
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
