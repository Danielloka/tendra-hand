// Tendra glove wire protocol: ESP-NOW packets between glove and dongle, and the USB CDC framing
// between dongle (or a glove in USB raw mode) and the PC.
//
// Shared by the glove firmware, the dongle firmware and the host tests; mirrored by
// tools/glove_reader.py (keep in sync; test/host/test_protocol.cpp prints the offsets).
//
// All multi-byte fields are little-endian (native on ESP32-S3 and x86). Structs are packed, so
// fields may be unaligned: copy them out (memcpy) rather than taking pointers on other CPUs.
// Every ESP-NOW packet starts with PacketHeader and ends with a CRC-16/CCITT-FALSE over all bytes
// before the CRC field.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc16.h"

namespace tg {

constexpr uint16_t kMagic = 0x4754;          // bytes 'T','G' on the wire (LE)
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kEspNowMaxPayload = 250;    // ESP_NOW_MAX_DATA_LEN (v1 ESP-NOW)

constexpr int kNumMag = 15;   // TMAG5273 3-axis Hall sensors
constexpr int kNumImu = 2;    // ICM-42688-P: [0] thumb metacarpal, [1] hand plate
constexpr int kNumSensors = kNumMag + kNumImu;

// Scale factors of the raw values in DataPacket (datasheet typicals; calibrate on the host).
constexpr float kMagLsbPerMt = 820.0f;     // TMAG5273x1, ±40 mT range (SLYS045C §5.7)
constexpr float kAccelLsbPerG = 8192.0f;   // ICM-42688-P ACCEL_FS_SEL = 2, ±4 g (DS-000347 §3.2)
constexpr float kGyroLsbPerDps = 32.8f;    // ICM-42688-P GYRO_FS_SEL = 1, ±1000 dps
constexpr float kImuTempLsbPerC = 132.48f; // °C = raw / 132.48 + 25
constexpr float kImuTempOffsetC = 25.0f;

// SYNC blink: start bit (always on) then the 4-bit code MSB first, kSyncBits slots of equal length.
constexpr int kSyncBits = 5;
inline bool sync_slot_on(uint8_t code, int slot) {
  if (slot == 0) return true;
  if (slot < 0 || slot >= kSyncBits) return false;
  return (code >> (kSyncBits - 1 - slot)) & 1u;
}

enum class Side : uint8_t { Left = 'L', Right = 'R', Unknown = '?' };

enum class PacketType : uint8_t {
  Data = 1,      // glove -> dongle, one sensor sweep
  Event = 2,     // glove -> dongle, MARK / SYNC edge / stream start/stop / battery / fault
  Hello = 3,     // glove -> broadcast, pairing request
  HelloAck = 4,  // dongle -> glove, pairing reply carrying the dongle MAC
  Command = 5,   // dongle -> glove
  CommandAck = 6 // glove -> dongle, reply to a Command (PING reply carries t_us)
};

enum class Command : uint8_t {
  Ping = 1,
  StartStream = 2,
  StopStream = 3,
  Sync = 4,      // arg = 4-bit code to blink on LED_SYNC
  SetRate = 5,   // arg = sample rate in Hz (50, 100 or 200)
  Reboot = 6,
};

enum class EventType : uint8_t {
  Boot = 1,
  Mark = 2,          // short button press: episode marker
  SyncEdge = 3,      // first (rising) edge of a SYNC blink; arg = code
  StreamStart = 4,
  StreamStop = 5,
  BatteryLow = 6,    // SOC < 15 % or VCELL < 3.50 V
  BatteryCritical = 7,  // VCELL < 3.30 V: glove shuts down
  SensorFault = 8,   // arg = sensor index, arg2 = 1 faulty / 0 recovered
  BusRecovery = 9,   // arg2 = number of recoveries so far (saturating)
  OverTemp = 10,     // IMU temperature > 45 °C (skin safety)
  DoubleTap = 11,    // reserved (IMU tap detection, not implemented yet)
};

// Status flag bits in DataPacket::status.
enum StatusBits : uint16_t {
  kStatusBusError = 1u << 0,       // at least one I2C error in this sweep
  kStatusLowBattery = 1u << 1,
  kStatusCriticalBattery = 1u << 2,
  kStatusCharging = 1u << 3,       // PGOOD_N low: USB power present (BQ24074 charging/charged)
  kStatusOverrun = 1u << 4,        // sweep longer than the period, or a timer tick was missed
  kStatusOverTemp = 1u << 5,
  kStatusFuelGaugeFault = 1u << 6, // MAX17048 not answering: battery fields are stale
  kStatusAnySaturated = 1u << 7,   // see mag_saturated
  kStatusBusRecovered = 1u << 8,   // a bus recovery ran just before this sweep
  kStatusStale = 1u << 9,          // at least one TMAG returned the same SET_COUNT twice
  kStatusUsbRaw = 1u << 10,        // frame is also being streamed over USB
};

#pragma pack(push, 1)

struct PacketHeader {
  uint16_t magic;     // kMagic
  uint8_t version;    // kProtocolVersion
  uint8_t type;       // PacketType
  uint8_t side;       // Side
  uint8_t reserved;   // 0 (Data: sample rate / 10 Hz, informational)
};
static_assert(sizeof(PacketHeader) == 6, "header layout");

struct DataPacket {
  PacketHeader hdr;
  uint32_t seq;            // sweep counter since boot (increments every sweep, streamed or not)
  uint64_t t_us;           // esp_timer_get_time() at the start of the sweep
  uint16_t sweep_us;       // duration of the sweep (I2C reads only)
  uint32_t sensor_ok;      // bit i = sensor i (SensorMap order) read OK in this sweep
  uint16_t mag_saturated;  // bit i = mag sensor i has an axis at full scale
  uint16_t status;         // StatusBits
  uint8_t batt_soc;        // %, 0..100 (255 = unknown)
  uint16_t batt_mv;        // cell voltage, mV (0 = unknown)
  uint16_t ring_drops;     // frames dropped because the radio ring was full (wraps)
  uint16_t tx_fail;        // ESP-NOW sends without MAC ACK (wraps)
  int16_t mag[kNumMag][3];  // raw TMAG5273 X, Y, Z (±40 mT range: 820 LSB/mT)
  int16_t imu[kNumImu][6];  // raw ICM-42688-P ax, ay, az (±4 g: 8192 LSB/g), gx, gy, gz (±1000 dps: 32.8 LSB/dps)
  int16_t imu_temp[kNumImu]; // raw TEMP_DATA: °C = raw / 132.48 + 25
  uint16_t crc;
};

struct EventPacket {
  PacketHeader hdr;
  uint32_t seq;      // seq of the sweep that was current when the event happened
  uint64_t t_us;     // esp_timer_get_time() of the event itself (SYNC: the LED edge)
  uint8_t event;     // EventType
  uint8_t arg;       // event-specific (SYNC code, sensor index...)
  uint16_t arg2;     // event-specific
  uint16_t crc;
};

struct HelloPacket {
  PacketHeader hdr;
  uint8_t mac[6];     // glove STA MAC
  uint16_t fw_version;  // major << 8 | minor
  uint8_t paired;     // 1 if the glove already has a stored dongle MAC
  uint16_t crc;
};

struct HelloAckPacket {
  PacketHeader hdr;   // side = side being acknowledged
  uint8_t dongle_mac[6];
  uint8_t channel;    // Wi-Fi channel the dongle listens on
  uint16_t crc;
};

struct CommandPacket {
  PacketHeader hdr;   // side = target glove
  uint8_t cmd;        // Command
  uint8_t cmd_id;     // echoed in the ack
  uint16_t arg;
  uint16_t crc;
};

struct CommandAckPacket {
  PacketHeader hdr;
  uint8_t cmd;
  uint8_t cmd_id;
  uint8_t result;     // 0 = OK, 1 = bad argument, 2 = unknown command
  uint8_t streaming;  // 1 if streaming after the command
  uint32_t seq;       // current sweep seq
  uint64_t t_us;      // glove time when the command was handled
  uint16_t crc;
};

#pragma pack(pop)

static_assert(sizeof(DataPacket) <= kEspNowMaxPayload, "DataPacket must fit one ESP-NOW frame");
static_assert(sizeof(DataPacket) < 250, "requirement: < 250 bytes");
static_assert(sizeof(EventPacket) <= kEspNowMaxPayload, "EventPacket size");
static_assert(sizeof(DataPacket) == 155, "DataPacket layout changed: update glove_reader.py");
static_assert(sizeof(EventPacket) == 24, "EventPacket layout changed: update glove_reader.py");
static_assert(sizeof(HelloPacket) == 17, "HelloPacket layout");
static_assert(sizeof(HelloAckPacket) == 15, "HelloAckPacket layout");
static_assert(sizeof(CommandPacket) == 12, "CommandPacket layout");
static_assert(sizeof(CommandAckPacket) == 24, "CommandAckPacket layout");
static_assert(offsetof(DataPacket, mag) == 35, "mag offset");

inline void fill_header(PacketHeader& h, PacketType type, Side side, uint8_t reserved = 0) {
  h.magic = kMagic;
  h.version = kProtocolVersion;
  h.type = static_cast<uint8_t>(type);
  h.side = static_cast<uint8_t>(side);
  h.reserved = reserved;
}

// Seals a packet: computes the CRC over all bytes before the trailing crc field.
template <typename P>
inline void seal(P& p) {
  p.crc = crc16_ccitt(&p, sizeof(P) - sizeof(uint16_t));
}

// Validates magic, version, length for the type, and CRC. Returns the packet type or 0.
inline uint8_t validate(const uint8_t* buf, size_t len) {
  if (len < sizeof(PacketHeader) + 2) return 0;
  PacketHeader h;
  memcpy(&h, buf, sizeof(h));
  if (h.magic != kMagic || h.version != kProtocolVersion) return 0;
  size_t expect = 0;
  switch (static_cast<PacketType>(h.type)) {
    case PacketType::Data: expect = sizeof(DataPacket); break;
    case PacketType::Event: expect = sizeof(EventPacket); break;
    case PacketType::Hello: expect = sizeof(HelloPacket); break;
    case PacketType::HelloAck: expect = sizeof(HelloAckPacket); break;
    case PacketType::Command: expect = sizeof(CommandPacket); break;
    case PacketType::CommandAck: expect = sizeof(CommandAckPacket); break;
    default: return 0;
  }
  if (len != expect) return 0;
  uint16_t crc;
  memcpy(&crc, buf + len - 2, 2);
  if (crc16_ccitt(buf, len - 2) != crc) return 0;
  return h.type;
}

// ---------------------------------------------------------------------------------------------
// USB CDC framing (dongle -> PC, and glove -> PC in USB raw mode):
//   COBS( UsbHeader | payload[len] ) 0x00
// PC -> dongle: COBS( UsbCommand ) 0x00
enum class UsbKind : uint8_t {
  GlovePacket = 1,   // payload = one validated ESP-NOW packet (any type)
  DongleStatus = 2,  // payload = DongleStatus
  Text = 3,          // payload = UTF-8 log line (no terminator)
};

#pragma pack(push, 1)
struct UsbHeader {
  uint8_t kind;      // UsbKind
  uint8_t side;      // Side of the glove that sent it ('?' for dongle frames)
  uint64_t rx_t_us;  // dongle esp_timer_get_time() at reception (glove raw mode: glove time)
  int8_t rssi;       // dBm (0 = unknown / USB raw mode)
  uint8_t len;       // payload length
};
static_assert(sizeof(UsbHeader) == 12, "UsbHeader layout");

struct DongleStatus {
  uint32_t uptime_ms;
  uint32_t rx_ok[2];       // [0] left, [1] right: valid packets received
  uint32_t rx_bad;         // CRC / magic / length failures
  uint32_t queue_drops;    // rx queue full (USB writer too slow)
  uint32_t usb_drops;      // USB TX buffer full
  uint8_t paired[2];       // 1 if the glove's MAC is known
  uint8_t channel;
};
static_assert(sizeof(DongleStatus) == 27, "DongleStatus layout");

struct UsbCommand {
  uint8_t magic;     // 'C'
  uint8_t target;    // 'L', 'R' or '*' (both)
  uint8_t cmd;       // Command
  uint16_t arg;
};
static_assert(sizeof(UsbCommand) == 5, "UsbCommand layout");
#pragma pack(pop)

constexpr size_t kUsbMaxFrame = sizeof(UsbHeader) + kEspNowMaxPayload;

}  // namespace tg
