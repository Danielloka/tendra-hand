// Tendra glove dongle: stock Seeed XIAO ESP32-S3. ESP-NOW receiver for both gloves, USB CDC
// bridge to the PC.
//
//   glove --ESP-NOW--> on_recv (Wi-Fi task: timestamp with esp_timer, validate, enqueue)
//         --> bridge task: answer HELLO, learn glove MACs, COBS-frame to USB
//   PC --COBS(UsbCommand)--> bridge task --> CommandPacket to one or both gloves
//
// USB output is binary only: COBS(UsbHeader | payload) 0x00 per frame (see protocol.h); log lines
// travel as UsbKind::Text frames so they never corrupt the stream.
#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>

#include "cobs.h"
#include "protocol.h"

namespace {

constexpr uint8_t kDongleChannel = 1;     // must match the gloves' kWifiChannel
constexpr int kPinLed = 21;               // XIAO ESP32-S3 user LED. VERIFY: GPIO21, active low
constexpr int kRxQueueLen = 64;           // 160 ms of 2 x 200 Hz traffic
constexpr uint32_t kStatusPeriodMs = 1000;

struct RxItem {
  uint64_t rx_t_us;
  uint8_t mac[6];
  int8_t rssi;
  uint8_t len;
  uint8_t data[tg::kEspNowMaxPayload];
};

QueueHandle_t g_rx = nullptr;
tg::DongleStatus g_status = {};
uint8_t g_glove_mac[2][6] = {};
bool g_glove_known[2] = {false, false};
uint8_t g_cmd_id = 0;
uint32_t g_last_rx_ms = 0;
std::atomic<uint32_t> g_queue_drops{0};   // written from the Wi-Fi task

int side_index(uint8_t side) {
  if (side == static_cast<uint8_t>(tg::Side::Left)) return 0;
  if (side == static_cast<uint8_t>(tg::Side::Right)) return 1;
  return -1;
}

void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint64_t t = static_cast<uint64_t>(esp_timer_get_time());   // first thing: arrival time
  if (len <= 0 || len > static_cast<int>(tg::kEspNowMaxPayload)) return;
  RxItem item;
  item.rx_t_us = t;
  memcpy(item.mac, info->src_addr, 6);
  item.rssi = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;
  item.len = static_cast<uint8_t>(len);
  memcpy(item.data, data, len);
  if (xQueueSend(g_rx, &item, 0) != pdTRUE) g_queue_drops.fetch_add(1);
}

void on_send(const esp_now_send_info_t*, esp_now_send_status_t) {}

bool ensure_peer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = kDongleChannel;
  p.ifidx = WIFI_IF_STA;
  p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) return false;
  esp_now_rate_config_t rc = {};
  rc.phymode = WIFI_PHY_MODE_11G;
  rc.rate = WIFI_PHY_RATE_6M;
  esp_now_set_peer_rate_config(mac, &rc);
  return true;
}

void usb_frame(tg::UsbKind kind, uint8_t side, uint64_t t_us, int8_t rssi, const uint8_t* payload, size_t len) {
  uint8_t raw[tg::kUsbMaxFrame];
  if (len > tg::kEspNowMaxPayload) return;
  tg::UsbHeader h;
  h.kind = static_cast<uint8_t>(kind);
  h.side = side;
  h.rx_t_us = t_us;
  h.rssi = rssi;
  h.len = static_cast<uint8_t>(len);
  memcpy(raw, &h, sizeof(h));
  memcpy(raw + sizeof(h), payload, len);
  uint8_t enc[tg::cobs_max_encoded(tg::kUsbMaxFrame) + 1];
  size_t n = tg::cobs_encode(raw, sizeof(h) + len, enc);
  enc[n++] = 0;
  if (Serial.availableForWrite() < static_cast<int>(n)) {   // PC not reading: drop, never block
    ++g_status.usb_drops;
    return;
  }
  Serial.write(enc, n);
}

void usb_text(const char* s) {
  usb_frame(tg::UsbKind::Text, '?', esp_timer_get_time(), 0, reinterpret_cast<const uint8_t*>(s), strlen(s));
}

void send_hello_ack(const uint8_t mac[6], uint8_t side) {
  if (!ensure_peer(mac)) return;
  tg::HelloAckPacket a;
  tg::fill_header(a.hdr, tg::PacketType::HelloAck, static_cast<tg::Side>(side));
  esp_wifi_get_mac(WIFI_IF_STA, a.dongle_mac);
  a.channel = kDongleChannel;
  tg::seal(a);
  esp_now_send(mac, reinterpret_cast<const uint8_t*>(&a), sizeof(a));
}

void learn_glove(int idx, const uint8_t mac[6]) {
  if (g_glove_known[idx] && memcmp(g_glove_mac[idx], mac, 6) == 0) return;
  memcpy(g_glove_mac[idx], mac, 6);
  g_glove_known[idx] = ensure_peer(mac);
  g_status.paired[idx] = g_glove_known[idx] ? 1 : 0;
  char line[64];
  snprintf(line, sizeof(line), "glove %c at %02X:%02X:%02X:%02X:%02X:%02X", idx == 0 ? 'L' : 'R', mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
  usb_text(line);
}

void send_command(int idx, uint8_t cmd, uint16_t arg) {
  if (!g_glove_known[idx]) return;
  tg::CommandPacket c;
  tg::fill_header(c.hdr, tg::PacketType::Command, idx == 0 ? tg::Side::Left : tg::Side::Right);
  c.cmd = cmd;
  c.cmd_id = ++g_cmd_id;
  c.arg = arg;
  tg::seal(c);
  esp_now_send(g_glove_mac[idx], reinterpret_cast<const uint8_t*>(&c), sizeof(c));
}

void handle_rx(const RxItem& it) {
  const uint8_t type = tg::validate(it.data, it.len);
  if (type == 0) {
    ++g_status.rx_bad;
    return;
  }
  const uint8_t side = it.data[4];
  const int idx = side_index(side);
  if (idx < 0) {
    ++g_status.rx_bad;
    return;
  }
  ++g_status.rx_ok[idx];
  g_last_rx_ms = millis();
  if (type == static_cast<uint8_t>(tg::PacketType::Hello)) {
    learn_glove(idx, it.mac);
    send_hello_ack(it.mac, side);
  } else {
    learn_glove(idx, it.mac);   // e.g. dongle rebooted while the glove kept its stored MAC
  }
  usb_frame(tg::UsbKind::GlovePacket, side, it.rx_t_us, it.rssi, it.data, it.len);
}

// PC -> dongle: COBS frames carrying UsbCommand.
uint8_t g_in[64];
size_t g_in_len = 0;
bool g_in_overflow = false;

void handle_usb_byte(uint8_t b) {
  if (b != 0) {
    if (g_in_len < sizeof(g_in)) {
      g_in[g_in_len++] = b;
    } else {
      g_in_overflow = true;
    }
    return;
  }
  uint8_t dec[sizeof(g_in)];
  const size_t n = g_in_overflow ? 0 : tg::cobs_decode(g_in, g_in_len, dec, sizeof(dec));
  g_in_len = 0;
  g_in_overflow = false;
  if (n != sizeof(tg::UsbCommand)) return;
  tg::UsbCommand c;
  memcpy(&c, dec, sizeof(c));
  if (c.magic != 'C') return;
  if (c.target == 'L' || c.target == '*') send_command(0, c.cmd, c.arg);
  if (c.target == 'R' || c.target == '*') send_command(1, c.cmd, c.arg);
}

void bridge_task(void*) {
  RxItem it;
  uint32_t last_status = 0;
  for (;;) {
    while (xQueueReceive(g_rx, &it, pdMS_TO_TICKS(2)) == pdTRUE) handle_rx(it);
    while (Serial.available() > 0) handle_usb_byte(static_cast<uint8_t>(Serial.read()));
    const uint32_t now = millis();
    digitalWrite(kPinLed, (now - g_last_rx_ms) < 50 ? LOW : HIGH);   // flicker while receiving
    if (now - last_status >= kStatusPeriodMs) {
      last_status = now;
      g_status.uptime_ms = now;
      g_status.channel = kDongleChannel;
      g_status.queue_drops = g_queue_drops.load();
      usb_frame(tg::UsbKind::DongleStatus, '?', esp_timer_get_time(), 0, reinterpret_cast<const uint8_t*>(&g_status),
                sizeof(g_status));
    }
  }
}

}  // namespace

void setup() {
  pinMode(kPinLed, OUTPUT);
  digitalWrite(kPinLed, HIGH);   // off
  Serial.begin(921600);
  Serial.setTxTimeoutMs(0);
  g_rx = xQueueCreate(kRxQueueLen, sizeof(RxItem));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(kDongleChannel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    for (;;) {
      usb_text("esp_now_init failed");
      delay(1000);
    }
  }
  esp_now_register_recv_cb(on_recv);
  esp_now_register_send_cb(on_send);
  xTaskCreatePinnedToCore(bridge_task, "bridge", 8192, nullptr, 10, nullptr, 1);
}

void loop() { vTaskDelete(nullptr); }
