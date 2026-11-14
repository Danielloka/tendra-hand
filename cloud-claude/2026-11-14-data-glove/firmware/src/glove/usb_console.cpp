#include "usb_console.h"

#include <esp_timer.h>
#include <stdarg.h>

#include "cobs.h"
#include "glove_config.h"
#include "glove_state.h"
#include "health.h"
#include "icm42688.h"
#include "radio.h"

UsbConsole g_console;

void UsbConsole::begin() {
  Serial.begin(921600);        // ignored by native USB CDC; kept for tools that need a value
  Serial.setTxTimeoutMs(0);    // never wait for the host inside write()
}

void UsbConsole::start() {
  xTaskCreatePinnedToCore(&UsbConsole::taskEntry, "console", 6144, this, kConsolePriority, nullptr,
                          kConsoleCore);
}

void UsbConsole::taskEntry(void* arg) { static_cast<UsbConsole*>(arg)->run(); }

void UsbConsole::writeFrame(uint8_t kind, const uint8_t* payload, size_t len, uint64_t t_us) {
  uint8_t raw[tg::kUsbMaxFrame];
  if (len > tg::kEspNowMaxPayload) return;
  tg::UsbHeader h;
  h.kind = kind;
  h.side = static_cast<uint8_t>(g_state.side);
  h.rx_t_us = t_us;
  h.rssi = 0;
  h.len = static_cast<uint8_t>(len);
  memcpy(raw, &h, sizeof(h));
  memcpy(raw + sizeof(h), payload, len);
  uint8_t enc[tg::cobs_max_encoded(tg::kUsbMaxFrame) + 1];
  size_t n = tg::cobs_encode(raw, sizeof(h) + len, enc);
  enc[n++] = 0x00;
  if (Serial.availableForWrite() < static_cast<int>(n)) {
    ++usb_drops_;
    return;
  }
  Serial.write(enc, n);
}

void UsbConsole::printf(const char* fmt, ...) {
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  const size_t len = static_cast<size_t>(n) < sizeof(buf) ? static_cast<size_t>(n) : sizeof(buf) - 1;
  if (g_state.usb_raw.load()) {
    // Strip the trailing newline: a Text frame is one line.
    size_t l = len;
    while (l && (buf[l - 1] == '\n' || buf[l - 1] == '\r')) --l;
    writeFrame(static_cast<uint8_t>(tg::UsbKind::Text), reinterpret_cast<uint8_t*>(buf), l,
               static_cast<uint64_t>(esp_timer_get_time()));
    return;
  }
  // Text mode: block at most a little (lowest priority task) rather than truncate replies.
  size_t off = 0;
  for (int tries = 0; off < len && tries < 50; ++tries) {
    const int room = Serial.availableForWrite();
    if (room > 0) {
      const size_t w = Serial.write(reinterpret_cast<const uint8_t*>(buf) + off,
                                    (len - off) < static_cast<size_t>(room) ? (len - off) : room);
      off += w;
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void UsbConsole::printHelp() {
  printf("Tendra glove %s - commands: i info, s scan, d dump frame, b battery, r USB raw stream,\n",
         GLOVE_FW_VERSION);
  printf("  t toggle streaming, p forget dongle, h help\n");
}

void UsbConsole::cmdInfo() {
  uint8_t mac[6];
  g_radio.ownMac(mac);
  printf("fw %s  proto v%u  side %c  mac %02X:%02X:%02X:%02X:%02X:%02X\n", GLOVE_FW_VERSION,
         tg::kProtocolVersion, static_cast<char>(g_state.side), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (g_radio.hasDongle()) {
    const uint8_t* d = g_radio.dongleMac();
    printf("dongle %02X:%02X:%02X:%02X:%02X:%02X  linked %d  ch %u\n", d[0], d[1], d[2], d[3], d[4], d[5],
           g_state.linked.load() ? 1 : 0, kWifiChannel);
  } else {
    printf("dongle: not paired (HELLO broadcasts on ch %u)\n", kWifiChannel);
  }
  printf("rate %lu Hz  streaming %d  usb_raw %d  seq %lu\n", (unsigned long)g_state.rate_hz.load(),
         g_state.streaming.load() ? 1 : 0, g_state.usb_raw.load() ? 1 : 0, (unsigned long)g_state.seq.load());
  printf("sweep last %lu us  max %lu us  overruns %lu  bus recoveries %lu\n",
         (unsigned long)g_state.last_sweep_us.load(), (unsigned long)g_state.max_sweep_us.load(),
         (unsigned long)g_state.overruns.load(), (unsigned long)g_state.bus_recoveries.load());
  printf("sensors ok 0x%05lX  faulty 0x%05lX  flags 0x%04X\n", (unsigned long)g_state.sensor_ok.load(),
         (unsigned long)g_state.faulty_mask.load(), g_state.health_flags.load());
  printf("radio tx ok %lu  fail %lu  ring drops %lu  usb drops %lu\n", (unsigned long)g_state.tx_ok.load(),
         (unsigned long)g_state.tx_fail.load(), (unsigned long)g_state.radio_ring.drops(),
         (unsigned long)(usb_drops_ + g_state.usb_ring.drops()));
}

void UsbConsole::cmdScan() {
  g_state.scan_done.store(false);
  g_state.scan_request.store(true);
  scan_pending_ = true;
  printf("scanning 9 x 112 addresses (a few per sweep)...\n");
}

static const char* name_at(int channel, uint8_t addr) {
  for (const auto& s : kSensorMap)
    if (s.channel == channel && s.addr == addr) return s.name;
  return nullptr;
}

void UsbConsole::cmdDump() {
  tg::DataPacket p;
  g_state.getSnapshot(p);
  printf("seq %lu  t %llu us  sweep %u us  ok 0x%05lX  sat 0x%04X  status 0x%04X  batt %u%% %u mV\n",
         (unsigned long)p.seq, (unsigned long long)p.t_us, p.sweep_us, (unsigned long)p.sensor_ok,
         p.mag_saturated, p.status, p.batt_soc, p.batt_mv);
  for (int i = 0; i < kSensorCount; ++i) {
    const SensorDesc& d = kSensorMap[i];
    const bool ok = p.sensor_ok & (1u << i);
    if (d.type == SensorType::Tmag5273) {
      printf("  %-11s ch%u 0x%02X %s  B = %8.3f %8.3f %8.3f mT\n", d.name, d.channel, d.addr, ok ? "ok " : "ERR",
             p.mag[d.slot][0] / tg::kMagLsbPerMt, p.mag[d.slot][1] / tg::kMagLsbPerMt,
             p.mag[d.slot][2] / tg::kMagLsbPerMt);
    } else {
      const int16_t* v = p.imu[d.slot];
      printf("  %-11s ch%u 0x%02X %s  a = %6.3f %6.3f %6.3f g  w = %7.1f %7.1f %7.1f dps  T = %.1f C\n", d.name,
             d.channel, d.addr, ok ? "ok " : "ERR", v[0] / tg::kAccelLsbPerG, v[1] / tg::kAccelLsbPerG,
             v[2] / tg::kAccelLsbPerG, v[3] / tg::kGyroLsbPerDps, v[4] / tg::kGyroLsbPerDps,
             v[5] / tg::kGyroLsbPerDps, Icm42688::tempC(p.imu_temp[d.slot]));
    }
  }
}

void UsbConsole::cmdBattery() {
  const Health::Battery b = g_health.battery();
  if (!b.valid) {
    printf("battery: fuel gauge not answering (USB power %s)\n", b.charging ? "present" : "absent");
    return;
  }
  printf("battery: %.1f %%  %u mV  %+.1f %%/h  status 0x%02X  USB power %s  flags 0x%04X\n", b.soc, b.mv,
         b.crate, b.status_hi, b.charging ? "present (charging/charged)" : "absent",
         g_state.health_flags.load());
}

void UsbConsole::handle(char c) {
  switch (c) {
    case 'i': cmdInfo(); break;
    case 's': cmdScan(); break;
    case 'd': cmdDump(); break;
    case 'b': cmdBattery(); break;
    case 'r': {
      const bool on = !g_state.usb_raw.load();
      if (!on) printf("usb raw off\n");
      g_state.usb_raw.store(on);
      if (on) printf("usb raw on (COBS frames)");
      break;
    }
    case 't': {
      const bool on = !g_state.streaming.load();
      g_state.streaming.store(on);
      post_event(on ? tg::EventType::StreamStart : tg::EventType::StreamStop);
      printf("streaming %s\n", on ? "on" : "off");
      break;
    }
    case 'p':
      g_radio.forgetPairing();
      printf("pairing forgotten; reboot to pair with a new dongle\n");
      break;
    case 'h':
    case '?': printHelp(); break;
    case '\r':
    case '\n':
    case ' ': break;
    default: printf("unknown command '%c' (h = help)\n", c); break;
  }
}

void UsbConsole::run() {
  tg::DataPacket p;
  tg::EventPacket ev;
  for (;;) {
    while (Serial.available() > 0) handle(static_cast<char>(Serial.read()));

    if (scan_pending_ && g_state.scan_done.load()) {
      scan_pending_ = false;
      for (int slot = 0; slot <= kMuxChannels; ++slot) {
        char line[160];
        int n = snprintf(line, sizeof(line), slot == 0 ? "upstream:" : "ch%d:", slot - 1);
        for (int a = 0; a < 128 && n < static_cast<int>(sizeof(line)) - 24; ++a) {
          if (!(g_state.scan_bits[slot][a / 32] & (1u << (a % 32)))) continue;
          const char* nm = slot > 0 ? name_at(slot - 1, static_cast<uint8_t>(a)) : nullptr;
          if (a == kMuxAddr) nm = "mux";
          n += snprintf(line + n, sizeof(line) - n, " 0x%02X%s%s", a, nm ? "=" : "", nm ? nm : "");
        }
        printf("%s\n", line);
      }
      // Sensors listed in the SensorMap but missing from the scan.
      for (const auto& s : kSensorMap) {
        if (!(g_state.scan_bits[s.channel + 1][s.addr / 32] & (1u << (s.addr % 32))))
          printf("  MISSING %s (ch%u 0x%02X)\n", s.name, s.channel, s.addr);
      }
    }

    if (g_state.usb_raw.load()) {
      while (g_state.usb_ring.pop(p))
        writeFrame(static_cast<uint8_t>(tg::UsbKind::GlovePacket), reinterpret_cast<const uint8_t*>(&p), sizeof(p),
                   p.t_us);
      while (xQueueReceive(g_state.usb_events, &ev, 0) == pdTRUE)
        writeFrame(static_cast<uint8_t>(tg::UsbKind::GlovePacket), reinterpret_cast<const uint8_t*>(&ev),
                   sizeof(ev), ev.t_us);
    } else {
      while (g_state.usb_ring.pop(p)) {}   // drain leftovers after raw mode was switched off
      xQueueReset(g_state.usb_events);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
