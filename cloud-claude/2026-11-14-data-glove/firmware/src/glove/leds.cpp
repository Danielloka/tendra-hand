#include "leds.h"

#include "protocol.h"

void Leds::begin() {
  pinMode(sync_pin_, OUTPUT);
  pinMode(status_pin_, OUTPUT);
  digitalWrite(sync_pin_, LOW);
  digitalWrite(status_pin_, LOW);
  esp_timer_create_args_t args = {};
  args.callback = &Leds::syncTimerCb;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "sync_led";
  esp_timer_create(&args, &sync_timer_);
}

void Leds::tick(uint32_t now_ms) {
  bool on = false;
  switch (pattern_) {
    case Pattern::Off: on = false; break;
    case Pattern::Unpaired: {
      const uint32_t t = now_ms % 1000;
      on = (t < 60) || (t >= 150 && t < 210);
      break;
    }
    case Pattern::Idle: on = (now_ms % 2000) < 50; break;
    case Pattern::Streaming: on = (now_ms % 1000) >= 60; break;
    case Pattern::LowBattery: on = (now_ms % 250) < 125; break;
    case Pattern::Fault: on = (now_ms % 1000) < 500; break;
  }
  digitalWrite(status_pin_, on ? HIGH : LOW);
}

bool Leds::startSync(uint8_t code, uint32_t bit_ms, SyncEdgeCallback cb, void* ctx) {
  int expected = -1;
  if (!sync_slot_.compare_exchange_strong(expected, 0)) return false;
  sync_code_ = code & 0x0F;
  sync_cb_ = cb;
  sync_ctx_ = ctx;
  // First edge right now (from this call), then one slot per period.
  digitalWrite(sync_pin_, HIGH);
  const int64_t t_edge = esp_timer_get_time();
  if (sync_cb_) sync_cb_(sync_code_, t_edge, sync_ctx_);
  esp_timer_start_periodic(sync_timer_, static_cast<uint64_t>(bit_ms) * 1000);
  return true;
}

void Leds::syncTimerCb(void* arg) {
  auto* self = static_cast<Leds*>(arg);
  const int slot = self->sync_slot_.load() + 1;
  if (slot >= tg::kSyncBits) {
    digitalWrite(self->sync_pin_, LOW);
    esp_timer_stop(self->sync_timer_);
    self->sync_slot_.store(-1);
    return;
  }
  digitalWrite(self->sync_pin_, tg::sync_slot_on(self->sync_code_, slot) ? HIGH : LOW);
  self->sync_slot_.store(slot);
}

void Leds::syncSolid(bool on) {
  if (!syncBusy()) digitalWrite(sync_pin_, on ? HIGH : LOW);
}

void Leds::allOff() {
  if (sync_timer_) esp_timer_stop(sync_timer_);
  sync_slot_.store(-1);
  digitalWrite(sync_pin_, LOW);
  digitalWrite(status_pin_, LOW);
}
