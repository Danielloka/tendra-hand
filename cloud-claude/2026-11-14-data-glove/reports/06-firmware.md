# 06 — Glove and dongle firmware (firmware agent, Opus)

Code: [`../firmware/`](../firmware/). PlatformIO `espressif32@8.0.0` (Arduino-ESP32 3.3 / IDF 5.5), C++17, one class per driver.

## Results

- `pio run -e glove`: success, RAM 16.8 %, flash 29.8 %. `pio run -e dongle`: success, RAM 13.8 %, flash 26.8 %. No project warnings (`-Wall -Wextra`). Re-run and confirmed by the main session.
- Host tests `test/host/test_protocol.cpp`: 40/40 (also ASan/UBSan/TSan clean): CRC-16/CCITT-FALSE, COBS (edge cases + 2000 random round trips), packet offsets, SYNC pattern, ring buffer incl. a 200k-frame two-thread stress test.
- `tools/glove_reader.py --check`: 14/14; recovers clock drift within 2 ppm and offset within 300 µs on a synthetic two-glove stream.
- **Not run on hardware.**

## Design

- Tasks: sampler on core 1 (priority 20, esp_timer 5 ms, the only I2C user); radio (10), health (5), USB console (2) on core 0.
- TMAG5273: continuous XYZ, ±40 mT, 16× averaging (1.25 ms per set), 1-byte read of X/Y/Z + CONV_STATUS. ICM-42688-P: ±4 g, ±1000 dps, 1 kHz LN, burst read. MAX17048 on its own 100 kHz bus.
- Packet: 155 bytes (magic, version, side, seq, t_us, sweep_us, sensor_ok, saturation, status flags, battery, drops, 15 × 3 mag, 2 × 6 IMU, temperatures, CRC). Dongle → PC: COBS frames with rx timestamp and RSSI.
- Radio: ESP-NOW at 6 Mb/s (~18 % airtime for two gloves at 200 Hz), ≤ 4 frames in flight, HELLO pairing stored in NVS, commands only from the paired dongle.
- Faults: 5 errors → sensor faulty + 1 s retries; 3 all-fail sweeps → 9-clock bus recovery + switch reset; battery < 3.30 V × 3 → stop, sensors to sleep, deep sleep; IMU temperature > 45 °C flag; 2 s task watchdog; SYNC LED blink with exact first-edge timestamp.
- Timing at 400 kHz: 6 switch selects 300 µs + 15 TMAG reads 2,775 µs + 2 IMU bursts 780 µs = **3.86 ms of 5 ms**; driver overhead unmeasured (100 Hz fallback).

## VERIFY on hardware

I2C driver overhead; ICM-42688-P values vs datasheet rev 1.9; TMAG saturation threshold and SET_COUNT; PGOOD/ALRT pull-ups; GPIO0 deep-sleep wake; XIAO LED pin; enable ESP-NOW encryption before field use; MAX17048 battery model.
