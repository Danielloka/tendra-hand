#include "sampler.h"

#include <esp_task_wdt.h>
#include <new>

namespace {
// Static storage for the drivers (no heap after boot).
alignas(Tmag5273) uint8_t g_tmag_storage[tg::kNumMag][sizeof(Tmag5273)];
alignas(Icm42688) uint8_t g_imu_storage[tg::kNumImu][sizeof(Icm42688)];

constexpr int kScanAddrsPerSweep = 4;   // ~4 x 50 µs extra per sweep while a scan runs
constexpr int kScanSlots = kMuxChannels + 1;

// Channel run [first, last] of the SensorMap rows (rows are grouped by channel).
struct ChannelRun {
  uint8_t channel;
  uint8_t first;
  uint8_t count;
};
ChannelRun g_runs[kMuxChannels];
int g_num_runs = 0;

void build_runs() {
  g_num_runs = 0;
  for (int i = 0; i < kSensorCount; ++i) {
    if (g_num_runs == 0 || g_runs[g_num_runs - 1].channel != kSensorMap[i].channel) {
      g_runs[g_num_runs++] = {kSensorMap[i].channel, static_cast<uint8_t>(i), 0};
    }
    ++g_runs[g_num_runs - 1].count;
  }
}
}  // namespace

Sampler::Sampler()
    : bus_(Wire, kPinSensSda, kPinSensScl, kSensorI2cHz, kI2cTimeoutMs),
      mux_(bus_, kMuxAddr, kPinMuxResetN) {
  for (int i = 0; i < kSensorCount; ++i) {
    const SensorDesc& d = kSensorMap[i];
    if (d.type == SensorType::Tmag5273) {
      tmag_[d.slot] = new (g_tmag_storage[d.slot]) Tmag5273(bus_, d.addr);
    } else {
      imu_[d.slot] = new (g_imu_storage[d.slot]) Icm42688(bus_, d.addr);
    }
  }
}

bool Sampler::begin() {
  build_runs();
  mux_.begin();
  if (!bus_.begin()) return false;
  // A brown-out or reset in the middle of a transfer can leave a slave holding SDA.
  bus_.recover();
  boot_ok_ = 0;
  const uint32_t now = millis();
  for (int i = 0; i < kSensorCount; ++i) {
    const bool ok = initSensor(i);
    st_[i].needs_init = !ok;
    st_[i].faulty = !ok;
    st_[i].next_retry_ms = now + kSensorRetryMs;
    boot_ok_ += ok ? 1 : 0;
  }
  uint32_t faulty = 0;
  for (int i = 0; i < kSensorCount; ++i) faulty |= st_[i].faulty ? (1u << i) : 0;
  g_state.faulty_mask.store(faulty);
  return true;
}

int Sampler::sensorsOk() const { return boot_ok_; }

bool Sampler::initSensor(int i) {
  const SensorDesc& d = kSensorMap[i];
  if (!mux_.select(static_cast<int8_t>(d.channel))) return false;
  if (d.type == SensorType::Tmag5273) return tmag_[d.slot]->init() == Tmag5273::InitResult::Ok;
  return imu_[d.slot]->init() == Icm42688::InitResult::Ok;
}

void Sampler::start() {
  xTaskCreatePinnedToCore(&Sampler::taskEntry, "sampler", 4096, this, kSamplerPriority, &task_,
                          kSamplerCore);
  esp_timer_create_args_t args = {};
  args.callback = &Sampler::timerCb;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;   // esp_timer task: high priority, not an ISR
  args.name = "sample_tick";
  esp_timer_create(&args, &timer_);
  period_us_ = 1000000 / g_state.rate_hz.load();
  esp_timer_start_periodic(timer_, period_us_);
}

void Sampler::timerCb(void* arg) {
  auto* self = static_cast<Sampler*>(arg);
  xTaskNotifyGive(self->task_);
}

void Sampler::taskEntry(void* arg) { static_cast<Sampler*>(arg)->run(); }

void Sampler::applyRate() {
  const uint32_t hz = g_state.rate_hz.load();
  period_us_ = 1000000 / hz;
  esp_timer_stop(timer_);
  esp_timer_start_periodic(timer_, period_us_);
}

void Sampler::run() {
  esp_task_wdt_add(nullptr);
  for (;;) {
    // Timeout so the watchdog is fed even when the timer is stopped (shutdown).
    const uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    esp_task_wdt_reset();
    if (g_state.shutdown_request.load()) {
      if (!g_state.shutdown_done.load()) {
        esp_timer_stop(timer_);
        shutdownSensors();
        g_state.shutdown_done.store(true);
      }
      continue;
    }
    if (g_state.rate_changed.exchange(false)) applyRate();
    if (ticks == 0) continue;
    sweep(ticks - 1);
  }
}

bool Sampler::readSensor(int i, tg::DataPacket& p, bool& saturated, bool& stale) {
  const SensorDesc& d = kSensorMap[i];
  if (d.type == SensorType::Tmag5273) {
    Tmag5273::Sample s;
    if (!tmag_[d.slot]->read(s)) return false;
    if (s.por || s.diag_fail) {
      st_[i].needs_init = true;   // lost its configuration (power glitch) or internal fault
      return false;
    }
    memcpy(p.mag[d.slot], s.xyz, sizeof(s.xyz));
    saturated = s.saturated;
    stale = s.stale;
    return true;
  }
  Icm42688::Sample s;
  if (!imu_[d.slot]->read(s)) return false;
  memcpy(&p.imu[d.slot][0], s.accel, sizeof(s.accel));
  memcpy(&p.imu[d.slot][3], s.gyro, sizeof(s.gyro));
  p.imu_temp[d.slot] = s.temp_raw;
  g_state.imu_temp_raw[d.slot].store(s.temp_raw);
  return true;
}

void Sampler::noteResult(int i, bool ok, uint32_t now_ms) {
  SensorState& s = st_[i];
  if (ok) {
    s.consecutive_errors = 0;
    return;
  }
  ++s.total_errors;
  if (s.consecutive_errors < 255) ++s.consecutive_errors;
  if (!s.faulty && (s.consecutive_errors >= kSensorFaultThreshold || s.needs_init)) {
    s.faulty = true;
    s.needs_init = true;
    s.next_retry_ms = now_ms + kSensorRetryMs;
    post_event(tg::EventType::SensorFault, static_cast<uint8_t>(i), 1);
  }
}

void Sampler::sweep(uint32_t missed_ticks) {
  const int64_t t0 = esp_timer_get_time();
  const uint32_t now_ms = static_cast<uint32_t>(t0 / 1000);
  const uint32_t seq = g_state.seq.load() + 1;

  tg::DataPacket p;
  memset(&p, 0, sizeof(p));
  tg::fill_header(p.hdr, tg::PacketType::Data, g_state.side,
                  static_cast<uint8_t>(g_state.rate_hz.load() / 10));
  p.seq = seq;
  p.t_us = static_cast<uint64_t>(t0);

  uint32_t ok_mask = 0;
  uint16_t sat_mask = 0;
  uint16_t status = 0;
  int attempted = 0, failed = 0;
  // SET_COUNT can only reveal a stuck TMAG when conversions per period is not a multiple of 8.
  const bool stale_meaningful = ((period_us_ / Tmag5273::kConversionUs) % 8) != 0;

  for (int r = 0; r < g_num_runs; ++r) {
    const ChannelRun& run = g_runs[forward_ ? r : (g_num_runs - 1 - r)];
    bool any_healthy = false;
    for (int k = 0; k < run.count; ++k) any_healthy |= !st_[run.first + k].faulty;
    if (!any_healthy) continue;
    if (!mux_.select(static_cast<int8_t>(run.channel))) {
      // The mux itself did not answer: every sensor on this channel counts as failed.
      for (int k = 0; k < run.count; ++k) {
        const int i = run.first + k;
        if (st_[i].faulty) continue;
        ++attempted;
        ++failed;
        noteResult(i, false, now_ms);
      }
      status |= tg::kStatusBusError;
      continue;
    }
    for (int k = 0; k < run.count; ++k) {
      const int i = run.first + k;
      if (st_[i].faulty) continue;
      ++attempted;
      bool sat = false, stale = false;
      const bool ok = readSensor(i, p, sat, stale);
      if (ok) {
        ok_mask |= 1u << i;
        if (sat) sat_mask |= static_cast<uint16_t>(1u << kSensorMap[i].slot);
        if (stale && stale_meaningful) status |= tg::kStatusStale;
      } else {
        ++failed;
        status |= tg::kStatusBusError;
      }
      noteResult(i, ok, now_ms);
    }
  }
  forward_ = !forward_;

  // Whole-bus failure -> recovery (rate limited).
  if (attempted > 0 && failed == attempted) {
    if (++all_fail_sweeps_ >= kAllFailSweepsBeforeRecovery &&
        (now_ms - last_recovery_ms_) >= kMinRecoveryIntervalMs) {
      recoverBus();
      all_fail_sweeps_ = 0;
      last_recovery_ms_ = now_ms;
    }
  } else {
    all_fail_sweeps_ = 0;
  }

  // At most one re-init of a faulty sensor per sweep (bounded extra time: TMAG ~0.6 ms, ICM ~2 ms).
  int reinits = 0;
  for (int i = 0; i < kSensorCount && reinits < kMaxReinitsPerSweep; ++i) {
    SensorState& s = st_[i];
    if (!s.faulty || static_cast<int32_t>(now_ms - s.next_retry_ms) < 0) continue;
    ++reinits;
    s.next_retry_ms = now_ms + kSensorRetryMs;
    if (initSensor(i)) {
      s.faulty = false;
      s.needs_init = false;
      s.consecutive_errors = 0;
      post_event(tg::EventType::SensorFault, static_cast<uint8_t>(i), 0);
    }
  }

  // Bus scan for the console, a few addresses per sweep.
  if (g_state.scan_request.exchange(false)) {
    memset(g_state.scan_bits, 0, sizeof(g_state.scan_bits));
    g_state.scan_done.store(false);
    scan_index_ = 0;
  }
  if (scan_index_ >= 0) scanSlice();

  const int64_t t1 = esp_timer_get_time();
  const uint32_t sweep_us = static_cast<uint32_t>(t1 - t0);
  if (sweep_us > period_us_ || missed_ticks > 0) {
    status |= tg::kStatusOverrun;
    g_state.overruns.fetch_add(missed_ticks > 0 ? missed_ticks : 1);
  }
  if (recovered_flag_) {
    status |= tg::kStatusBusRecovered;
    recovered_flag_ = false;
  }
  if (sat_mask) status |= tg::kStatusAnySaturated;
  status |= g_state.health_flags.load();
  if (g_state.usb_raw.load()) status |= tg::kStatusUsbRaw;

  p.sweep_us = static_cast<uint16_t>(sweep_us > 0xFFFF ? 0xFFFF : sweep_us);
  p.sensor_ok = ok_mask;
  p.mag_saturated = sat_mask;
  p.status = status;
  p.batt_soc = g_state.batt_soc.load();
  p.batt_mv = g_state.batt_mv.load();
  p.ring_drops = static_cast<uint16_t>(g_state.radio_ring.drops());
  p.tx_fail = static_cast<uint16_t>(g_state.tx_fail.load());
  tg::seal(p);

  if (g_state.streaming.load() && g_state.linked.load()) {
    g_state.radio_ring.push(p);
    if (g_state.radio_task) xTaskNotifyGive(g_state.radio_task);
  }
  if (g_state.usb_raw.load()) g_state.usb_ring.push(p);
  g_state.putSnapshot(p);

  uint32_t faulty = 0;
  for (int i = 0; i < kSensorCount; ++i) faulty |= st_[i].faulty ? (1u << i) : 0;
  g_state.faulty_mask.store(faulty);
  g_state.sensor_ok.store(ok_mask);
  g_state.seq.store(seq);
  g_state.sweeps.fetch_add(1);
  g_state.last_sweep_us.store(sweep_us);
  if (sweep_us > g_state.max_sweep_us.load()) g_state.max_sweep_us.store(sweep_us);
}

void Sampler::recoverBus() {
  g_state.bus_recoveries.fetch_add(1);
  bus_.recover();
  mux_.hardwareReset();   // disconnects all downstream channels
  // A slave stuck on one branch only shows when that branch is connected: check each.
  for (int ch = 0; ch < kMuxChannels; ++ch) {
    if (!mux_.select(static_cast<int8_t>(ch))) break;
    bus_.end();
    pinMode(kPinSensSda, INPUT);
    const bool stuck = digitalRead(kPinSensSda) == LOW;
    bus_.begin();
    if (stuck) bus_.recover();
    mux_.invalidate();
  }
  mux_.deselectAll();
  for (int i = 0; i < kSensorCount; ++i) {
    st_[i].needs_init = true;
    st_[i].faulty = true;               // re-init through the normal 1 s retry path...
    st_[i].next_retry_ms = 0;           // ...starting immediately, one sensor per sweep
  }
  recovered_flag_ = true;
  const uint32_t n = g_state.bus_recoveries.load();
  post_event(tg::EventType::BusRecovery, 0, static_cast<uint16_t>(n > 0xFFFF ? 0xFFFF : n));
}

void Sampler::scanSlice() {
  for (int k = 0; k < kScanAddrsPerSweep && scan_index_ >= 0; ++k) {
    const int slot = scan_index_ / 128;
    const uint8_t addr = static_cast<uint8_t>(scan_index_ % 128);
    if (slot >= kScanSlots) {
      scan_index_ = -1;
      g_state.scan_done.store(true);
      return;
    }
    const int8_t ch = static_cast<int8_t>(slot - 1);   // slot 0 = upstream (no channel)
    if (mux_.select(ch) && addr >= 0x08 && addr <= 0x77 && bus_.probe(addr)) {
      g_state.scan_bits[slot][addr / 32] |= 1u << (addr % 32);
    }
    ++scan_index_;
  }
}

void Sampler::shutdownSensors() {
  // TMAG5273 -> sleep mode (5 nA typ), ICM-42688-P -> accel/gyro/temp off (OFF state).
  for (int r = 0; r < g_num_runs; ++r) {
    if (!mux_.select(static_cast<int8_t>(g_runs[r].channel))) continue;
    for (int k = 0; k < g_runs[r].count; ++k) {
      const SensorDesc& d = kSensorMap[g_runs[r].first + k];
      if (d.type == SensorType::Tmag5273) {
        tmag_[d.slot]->sleep();
      } else {
        imu_[d.slot]->powerOff();
      }
    }
  }
  mux_.deselectAll();
}
