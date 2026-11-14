// Radio: ESP-NOW link from the glove to the dongle (task on core 0).
//
// Pairing: while no dongle is linked, the glove broadcasts HELLO (side, own MAC, fw version) every
// 500 ms on kWifiChannel. The dongle answers with HELLO_ACK carrying its MAC; the glove adds it
// as a unicast peer and stores it in NVS (Preferences "tglove"/"dongle"), so after a reboot it
// streams immediately. If kTxFailsBeforeRepair consecutive unicast sends get no MAC-level ACK,
// the link counts as lost and HELLO broadcasts resume (the stored MAC is kept until a different
// dongle answers or the console command `p` forgets it).
//
// PHY rate: unicast frames to the dongle go out at 6 Mbit/s OFDM (esp_now_set_peer_rate_config)
// instead of the 1 Mbit/s default. A 155-byte data packet is ~198 bytes on air (ESP-NOW action
// frame header + FCS); with preamble, SIFS + ACK and average DIFS/backoff that is ~0.45 ms per
// frame at 6 Mbit/s versus ~2.4 ms at 1 Mbit/s, so 2 gloves x 200 Hz use ~18 % of the channel
// instead of ~100 % (estimate; measure with the dongle's counters).
//
// Data path: the sampler pushes sealed DataPackets into g_state.radio_ring (lock-free) and
// notifies this task; events come through g_state.radio_events. At most kMaxInFlight sends are
// outstanding (counted down by the send callback), so esp_now_send never returns ESP_ERR_ESPNOW_NO_MEM
// in normal operation; if the ring fills anyway the sampler drops the newest frame and counts it.
#pragma once

#include <Arduino.h>
#include <atomic>

#include "protocol.h"

class Radio {
 public:
  using CommandHandler = void (*)(const tg::CommandPacket& cmd);

  bool begin(CommandHandler handler);
  void start();
  void forgetPairing();
  bool hasDongle() const { return have_dongle_; }
  const uint8_t* dongleMac() const { return dongle_mac_; }
  void ownMac(uint8_t mac[6]) const;
  void sendAck(const tg::CommandAckPacket& ack);   // from the command handler (radio task)

 private:
  static void taskEntry(void* arg);
  void run();
  void handleRx();
  void sendHello();
  bool sendRaw(const uint8_t* mac, const void* data, size_t len);
  bool addPeer(const uint8_t mac[6], bool set_rate);
  void setDongle(const uint8_t mac[6], bool persist);

  CommandHandler handler_ = nullptr;
  uint8_t dongle_mac_[6] = {};
  bool have_dongle_ = false;
  uint32_t last_hello_ms_ = 0;
};

extern Radio g_radio;
