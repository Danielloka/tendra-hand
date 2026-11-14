#include "radio.h"

#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <freertos/queue.h>

#include "glove_config.h"
#include "glove_state.h"

Radio g_radio;

namespace {
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct RxItem {
  uint8_t mac[6];
  uint8_t len;
  uint8_t data[32];   // only small control packets are accepted from the dongle
};
QueueHandle_t g_rx_queue = nullptr;
std::atomic<int> g_in_flight{0};
std::atomic<uint32_t> g_consecutive_fails{0};

void on_send(const esp_now_send_info_t* info, esp_now_send_status_t status) {
  (void)info;
  if (g_in_flight.load() > 0) g_in_flight.fetch_sub(1);
  if (status == ESP_NOW_SEND_SUCCESS) {
    g_state.tx_ok.fetch_add(1);
    g_consecutive_fails.store(0);
  } else {
    g_state.tx_fail.fetch_add(1);
    g_consecutive_fails.fetch_add(1);
  }
  if (g_state.radio_task) xTaskNotifyGive(g_state.radio_task);
}

void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  // Wi-Fi task context: validate cheaply, copy, hand over. Never block here.
  if (len <= 0 || len > static_cast<int>(sizeof(RxItem::data))) return;
  RxItem item;
  memcpy(item.mac, info->src_addr, 6);
  item.len = static_cast<uint8_t>(len);
  memcpy(item.data, data, len);
  if (xQueueSend(g_rx_queue, &item, 0) == pdTRUE && g_state.radio_task)
    xTaskNotifyGive(g_state.radio_task);
}
}  // namespace

void Radio::ownMac(uint8_t mac[6]) const { esp_wifi_get_mac(WIFI_IF_STA, mac); }

bool Radio::begin(CommandHandler handler) {
  handler_ = handler;
  g_rx_queue = xQueueCreate(8, sizeof(RxItem));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // no modem sleep: latency and ACK reliability over current
  if (esp_wifi_set_channel(kWifiChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(on_send);
  esp_now_register_recv_cb(on_recv);
  addPeer(kBroadcast, false);

  Preferences prefs;
  if (prefs.begin("tglove", true)) {
    uint8_t mac[6];
    if (prefs.getBytes("dongle", mac, 6) == 6) setDongle(mac, false);
    prefs.end();
  }
  // Link is assumed up with a stored MAC; the first failed sends drop it back to pairing.
  g_state.linked.store(have_dongle_);
  return true;
}

bool Radio::addPeer(const uint8_t mac[6], bool set_rate) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = kWifiChannel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;   // VERIFY/TODO: enable ESP-NOW encryption (PMK/LMK) before field use
    if (esp_now_add_peer(&peer) != ESP_OK) return false;
  }
  if (set_rate) {
    esp_now_rate_config_t rc = {};
    rc.phymode = WIFI_PHY_MODE_11G;
    rc.rate = WIFI_PHY_RATE_6M;
    rc.ersu = false;
    rc.dcm = false;
    esp_now_set_peer_rate_config(mac, &rc);
  }
  return true;
}

void Radio::setDongle(const uint8_t mac[6], bool persist) {
  if (have_dongle_ && memcmp(mac, dongle_mac_, 6) != 0 && esp_now_is_peer_exist(dongle_mac_))
    esp_now_del_peer(dongle_mac_);
  memcpy(dongle_mac_, mac, 6);
  have_dongle_ = addPeer(mac, true);
  if (persist && have_dongle_) {
    Preferences prefs;
    if (prefs.begin("tglove", false)) {
      uint8_t old[6];
      if (prefs.getBytes("dongle", old, 6) != 6 || memcmp(old, mac, 6) != 0) prefs.putBytes("dongle", mac, 6);
      prefs.end();
    }
  }
}

void Radio::forgetPairing() {
  Preferences prefs;
  if (prefs.begin("tglove", false)) {
    prefs.remove("dongle");
    prefs.end();
  }
  g_state.linked.store(false);
}

void Radio::start() {
  xTaskCreatePinnedToCore(&Radio::taskEntry, "radio", 4096, this, kRadioPriority, &g_state.radio_task,
                          kRadioCore);
}

void Radio::taskEntry(void* arg) { static_cast<Radio*>(arg)->run(); }

bool Radio::sendRaw(const uint8_t* mac, const void* data, size_t len) {
  if (g_in_flight.load() >= kMaxInFlight) return false;
  g_in_flight.fetch_add(1);
  if (esp_now_send(mac, static_cast<const uint8_t*>(data), len) != ESP_OK) {
    g_in_flight.fetch_sub(1);
    g_state.tx_fail.fetch_add(1);
    return false;
  }
  return true;
}

void Radio::sendAck(const tg::CommandAckPacket& ack) {
  if (have_dongle_) sendRaw(dongle_mac_, &ack, sizeof(ack));
}

void Radio::sendHello() {
  tg::HelloPacket h;
  tg::fill_header(h.hdr, tg::PacketType::Hello, g_state.side);
  ownMac(h.mac);
  h.fw_version = kGloveFwVersion;
  h.paired = have_dongle_ ? 1 : 0;
  tg::seal(h);
  sendRaw(kBroadcast, &h, sizeof(h));
}

void Radio::handleRx() {
  RxItem item;
  while (xQueueReceive(g_rx_queue, &item, 0) == pdTRUE) {
    const uint8_t type = tg::validate(item.data, item.len);
    if (type == static_cast<uint8_t>(tg::PacketType::HelloAck)) {
      tg::HelloAckPacket a;
      memcpy(&a, item.data, sizeof(a));
      if (a.hdr.side != static_cast<uint8_t>(g_state.side)) continue;   // for the other glove
      setDongle(a.dongle_mac, true);
      g_consecutive_fails.store(0);
      g_state.linked.store(have_dongle_);
    } else if (type == static_cast<uint8_t>(tg::PacketType::Command)) {
      // Commands are accepted only from the paired dongle.
      if (!have_dongle_ || memcmp(item.mac, dongle_mac_, 6) != 0) continue;
      tg::CommandPacket c;
      memcpy(&c, item.data, sizeof(c));
      if (c.hdr.side != static_cast<uint8_t>(g_state.side) && c.hdr.side != '*') continue;
      g_consecutive_fails.store(0);
      g_state.linked.store(true);
      if (handler_) handler_(c);
    }
  }
}

void Radio::run() {
  esp_task_wdt_add(nullptr);
  tg::DataPacket frame;
  tg::EventPacket ev;
  bool have_frame = false;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    esp_task_wdt_reset();
    handleRx();

    const uint32_t now = millis();
    if (g_consecutive_fails.load() >= kTxFailsBeforeRepair && g_state.linked.load()) {
      g_state.linked.store(false);   // dongle gone: back to HELLO broadcasts
    }
    if (!g_state.linked.load()) {
      if (now - last_hello_ms_ >= kHelloPeriodMs) {
        last_hello_ms_ = now;
        sendHello();
      }
      // Discard backlog that can no longer be delivered (the sampler stops pushing while unlinked).
      while (g_state.radio_ring.pop(frame)) {}
      have_frame = false;
      xQueueReset(g_state.radio_events);
      continue;
    }

    // Events first (rare, small, time-relevant), then frames oldest-first.
    while (g_in_flight.load() < kMaxInFlight && xQueuePeek(g_state.radio_events, &ev, 0) == pdTRUE) {
      if (!sendRaw(dongle_mac_, &ev, sizeof(ev))) break;
      xQueueReceive(g_state.radio_events, &ev, 0);
    }
    while (g_in_flight.load() < kMaxInFlight) {
      if (!have_frame && !g_state.radio_ring.pop(frame)) break;
      have_frame = true;
      if (!sendRaw(dongle_mac_, &frame, sizeof(frame))) break;
      have_frame = false;
    }
  }
}
