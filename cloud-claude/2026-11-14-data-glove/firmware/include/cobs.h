// COBS (Consistent Overhead Byte Stuffing) for the USB CDC links. Every frame on the wire is
// cobs(payload) followed by one 0x00 delimiter, so a reader that joins mid-stream resyncs at the
// next zero. Worst-case overhead: 1 byte per 254 payload bytes (+1).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace tg {

constexpr size_t cobs_max_encoded(size_t n) { return n + n / 254 + 1; }

// Encodes `len` bytes into `out` (capacity >= cobs_max_encoded(len)). Does NOT append the 0x00
// delimiter. Returns the encoded length.
inline size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read = 0, write = 1, code_pos = 0;
  uint8_t code = 1;
  while (read < len) {
    if (in[read] == 0) {
      out[code_pos] = code;
      code = 1;
      code_pos = write++;
      ++read;
    } else {
      out[write++] = in[read++];
      if (++code == 0xFF) {
        out[code_pos] = code;
        code = 1;
        if (read == len) return write;   // canonical: no empty block after a full 254-byte run
        code_pos = write++;
      }
    }
  }
  out[code_pos] = code;
  return write;
}

// Decodes one frame (without its 0x00 delimiter). Returns the decoded length, or 0 on a malformed
// frame or if it would not fit in `out_cap`.
inline size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap) {
  size_t read = 0, write = 0;
  while (read < len) {
    uint8_t code = in[read];
    if (code == 0 || read + code > len) return 0;  // code byte points past the frame end
    ++read;
    for (uint8_t i = 1; i < code; ++i) {
      if (read >= len || write >= out_cap || in[read] == 0) return 0;
      out[write++] = in[read++];
    }
    if (code != 0xFF && read < len) {
      if (write >= out_cap) return 0;
      out[write++] = 0;
    }
  }
  return write;
}

}  // namespace tg
