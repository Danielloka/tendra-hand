// UsbConsole: human-readable diagnostics on the native USB CDC port (lowest-priority task, core 0).
//   i  info: firmware, side, MACs, link, rate, counters
//   s  bus scan per mux channel (runs a few addresses per sweep inside the sampler: ~1.5 s)
//   d  dump the latest frame, decoded (mT, g, dps, °C)
//   b  battery
//   r  toggle raw streaming over USB: binary COBS frames, the same format the dongle sends
//      (tools/glove_reader.py reads both); text replies are then wrapped as Text frames
//   t  toggle streaming (same as the 2 s button press)
//   p  forget the paired dongle
//   h  help
// Output never blocks the sampler: frames come through a separate lock-free ring and are dropped
// when the USB TX buffer is full (counted, shown by `i`).
#pragma once

#include <Arduino.h>

class UsbConsole {
 public:
  void begin();
  void start();

 private:
  static void taskEntry(void* arg);
  void run();
  void handle(char c);
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void writeFrame(uint8_t kind, const uint8_t* payload, size_t len, uint64_t t_us);
  void cmdInfo();
  void cmdScan();
  void cmdDump();
  void cmdBattery();
  void printHelp();
  bool scan_pending_ = false;
  uint32_t usb_drops_ = 0;
};

extern UsbConsole g_console;
