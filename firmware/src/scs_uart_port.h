// ScsPort on an ESP32 hardware UART, wired to the FE-URT-1 signal converter.
//
// The FE-URT-1 turns the MCU's separate TX/RX lines into the servos' single half-duplex data line
// and switches direction automatically, so no direction pin is needed.
#pragma once

#include <Arduino.h>

#include "scs_bus.h"

class ScsUartPort : public ScsPort {
 public:
  explicit ScsUartPort(HardwareSerial& uart) : uart_(uart) {}

  void begin(uint32_t baud, int rx_pin, int tx_pin) {
    uart_.setRxBufferSize(512);
    uart_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  }

  void write(const uint8_t* data, size_t n) override {
    uart_.write(data, n);
    uart_.flush();  // wait until the last byte is on the wire, so the reply timeout starts right
  }
  int read() override { return uart_.read(); }
  void clearInput() override {
    while (uart_.available() > 0) uart_.read();
  }
  uint32_t micros() override { return ::micros(); }

 private:
  HardwareSerial& uart_;
};
