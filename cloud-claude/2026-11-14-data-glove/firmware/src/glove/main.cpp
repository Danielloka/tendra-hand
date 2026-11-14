// Tendra glove firmware: wireless data glove with 15 TMAG5273 Hall sensors and 2 ICM-42688-P IMUs
// behind a TCA9548A switch, streaming 200 Hz frames over ESP-NOW to a USB dongle.
//
// Tasks:
//   core 1  sampler  (prio 20)  esp_timer 5 ms -> notify -> sweep all sensors -> DataPacket
//   core 0  radio    (prio 10)  ESP-NOW send/receive, pairing, command handling
//   core 0  health   (prio 5)   10 ms tick: button, LEDs, battery, temperature, deep sleep
//   core 0  console  (prio 2)   USB CDC diagnostics and USB raw streaming
// The Arduino loop task is deleted after setup().
//
// Safety: nothing here drives a load at boot except the two indicator LEDs (off). The task
// watchdog (2 s, panic + reboot) watches sampler, radio and health. Brown-out detection stays at
// its default (enabled).
#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include "glove_config.h"
#include "glove_state.h"
#include "health.h"
#include "radio.h"
#include "sampler.h"
#include "usb_console.h"

GloveState g_state;
static Sampler g_sampler;

void post_event(tg::EventType type, uint8_t arg, uint16_t arg2, int64_t t_us) {
  tg::EventPacket e;
  tg::fill_header(e.hdr, tg::PacketType::Event, g_state.side);
  e.seq = g_state.seq.load();
  e.t_us = static_cast<uint64_t>(t_us >= 0 ? t_us : esp_timer_get_time());
  e.event = static_cast<uint8_t>(type);
  e.arg = arg;
  e.arg2 = arg2;
  tg::seal(e);
  if (g_state.radio_events) {
    xQueueSend(g_state.radio_events, &e, 0);
    if (g_state.radio_task) xTaskNotifyGive(g_state.radio_task);
  }
  if (g_state.usb_events && g_state.usb_raw.load()) xQueueSend(g_state.usb_events, &e, 0);
}

static void on_sync_edge(uint8_t code, int64_t t_us, void*) {
  // seq = the sweep in progress / last completed when the LED went on; t_us is the edge itself.
  post_event(tg::EventType::SyncEdge, code, 0, t_us);
}

static bool rate_allowed(uint32_t hz) {
  for (uint32_t r : kAllowedRatesHz)
    if (r == hz) return true;
  return false;
}

// Runs in the radio task.
static void handle_command(const tg::CommandPacket& c) {
  tg::CommandAckPacket a;
  tg::fill_header(a.hdr, tg::PacketType::CommandAck, g_state.side);
  a.cmd = c.cmd;
  a.cmd_id = c.cmd_id;
  a.result = 0;
  bool reboot = false;
  switch (static_cast<tg::Command>(c.cmd)) {
    case tg::Command::Ping:
      break;
    case tg::Command::StartStream:
      if (!g_state.streaming.exchange(true)) post_event(tg::EventType::StreamStart);
      break;
    case tg::Command::StopStream:
      if (g_state.streaming.exchange(false)) post_event(tg::EventType::StreamStop);
      break;
    case tg::Command::Sync:
      if (!g_health.leds().startSync(static_cast<uint8_t>(c.arg & 0x0F), kSyncBitMs, on_sync_edge, nullptr))
        a.result = 1;   // a blink is already running
      break;
    case tg::Command::SetRate:
      if (rate_allowed(c.arg)) {
        g_state.rate_hz.store(c.arg);
        g_state.rate_changed.store(true);
      } else {
        a.result = 1;
      }
      break;
    case tg::Command::Reboot:
      reboot = true;
      break;
    default:
      a.result = 2;
      break;
  }
  a.streaming = g_state.streaming.load() ? 1 : 0;
  a.seq = g_state.seq.load();
  a.t_us = static_cast<uint64_t>(esp_timer_get_time());
  tg::seal(a);
  g_radio.sendAck(a);
  if (reboot) {
    g_state.streaming.store(false);
    vTaskDelay(pdMS_TO_TICKS(50));   // let the ack go out
    esp_restart();
  }
}

static void configure_task_wdt() {
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = kTaskWdtTimeoutMs;
  cfg.idle_core_mask = 0;   // only the tasks that subscribe themselves are watched
  cfg.trigger_panic = true; // panic -> reboot (brown-out and panic reset reasons are kept)
  if (esp_task_wdt_reconfigure(&cfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_init(&cfg);
}

void setup() {
  // LEDs off before anything else; the mux is held out of reset by Tca9548a::begin().
  pinMode(kPinLedSync, OUTPUT);
  digitalWrite(kPinLedSync, LOW);
  pinMode(kPinLedStatus, OUTPUT);
  digitalWrite(kPinLedStatus, LOW);

  // Woken from the protective deep sleep by the button? Re-check the cell before doing anything.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 && Health::cellTooLowToRun()) {
    Health::enterProtectiveSleep();
  }

  pinMode(kPinSideSel, INPUT_PULLUP);
  delayMicroseconds(50);
  g_state.side = digitalRead(kPinSideSel) == LOW ? tg::Side::Left : tg::Side::Right;
  pinMode(kPinHandImuInt1, INPUT);

  g_state.radio_events = xQueueCreate(16, sizeof(tg::EventPacket));
  g_state.usb_events = xQueueCreate(16, sizeof(tg::EventPacket));

  g_console.begin();
  configure_task_wdt();

  g_health.begin();     // fuel gauge failure is reported via flags, not fatal
  g_sampler.begin();    // sensors that fail init are retried every second
  g_radio.begin(handle_command);

  g_sampler.start();
  g_radio.start();
  g_health.start();
  g_console.start();
  post_event(tg::EventType::Boot, static_cast<uint8_t>(g_sampler.sensorsOk()),
             static_cast<uint16_t>(esp_reset_reason()));
}

void loop() { vTaskDelete(nullptr); }
