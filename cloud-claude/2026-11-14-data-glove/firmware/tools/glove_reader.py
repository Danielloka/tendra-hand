#!/usr/bin/env python3
"""Tendra glove USB reader: decodes the dongle's COBS stream (or a glove in USB raw mode).

Prints, every --interval seconds and per glove: frame rate, lost frames (seq gaps), CRC failures,
the glove's own counters (ring drops, ESP-NOW send failures), RSSI, sweep time, and the glove
clock vs the dongle clock (offset + drift), estimated from the LOWER ENVELOPE of
    d = rx_t_us (dongle) - t_us (glove)
Radio/USB latency only ever adds to d, so the minimum of d in each time bin is the sample with the
least queuing delay; a straight-line fit through those minima gives the clock offset (plus the
constant minimum latency) and the drift. Frames are kept as numpy arrays (mag: N x 15 x 3,
imu: N x 2 x 6), see GloveLog.

Usage:
  python tools/glove_reader.py --port /dev/ttyACM0          # or COM5 on Windows
  python tools/glove_reader.py --port auto --send '*:start'  # commands: start, stop, ping,
                                                             # sync:<0-15>, rate:<50|100|200>, reboot
  python tools/glove_reader.py --file .pio/host_frames.bin   # decode a captured stream (tests)
  python tools/glove_reader.py --port auto --save run.npz    # write arrays on exit

Layout constants mirror firmware/include/protocol.h (checked by test/host/test_protocol.cpp,
which writes a synthetic stream this script decodes in --file --check mode).
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field

import numpy as np

MAGIC = 0x4754
PROTO_VERSION = 1
N_MAG, N_IMU = 15, 2
MAG_LSB_PER_MT = 820.0
ACCEL_LSB_PER_G = 8192.0
GYRO_LSB_PER_DPS = 32.8

# protocol.h, packed little-endian
HDR = struct.Struct("<HBBBB")  # magic, version, type, side, reserved
DATA = struct.Struct("<HBBBB I Q H I H H B H H H 45h 12h 2h H".replace(" ", ""))
EVENT = struct.Struct("<HBBBB I Q B B H H".replace(" ", ""))
ACK = struct.Struct("<HBBBB B B B B I Q H".replace(" ", ""))
USB_HDR = struct.Struct("<BBQbB")  # kind, side, rx_t_us, rssi, len
STATUS = struct.Struct("<I II I I I BB B".replace(" ", ""))
assert DATA.size == 155 and EVENT.size == 24 and ACK.size == 24 and USB_HDR.size == 12 and STATUS.size == 27

T_DATA, T_EVENT, T_HELLO, T_HELLOACK, T_CMD, T_CMDACK = 1, 2, 3, 4, 5, 6
K_GLOVE, K_STATUS, K_TEXT = 1, 2, 3
EVENTS = {
    1: "BOOT",
    2: "MARK",
    3: "SYNC_EDGE",
    4: "STREAM_START",
    5: "STREAM_STOP",
    6: "BATTERY_LOW",
    7: "BATTERY_CRITICAL",
    8: "SENSOR_FAULT",
    9: "BUS_RECOVERY",
    10: "OVER_TEMP",
    11: "DOUBLE_TAP",
}
COMMANDS = {"ping": 1, "start": 2, "stop": 3, "sync": 4, "rate": 5, "reboot": 6}
STATUS_BITS = [
    "bus_err",
    "low_batt",
    "crit_batt",
    "charging",
    "overrun",
    "over_temp",
    "fg_fault",
    "saturated",
    "bus_recovered",
    "stale",
    "usb_raw",
]


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), same as firmware/include/crc16.h."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


assert crc16_ccitt(b"123456789") == 0x29B1


def cobs_decode(frame: bytes) -> bytes | None:
    out = bytearray()
    i, n = 0, len(frame)
    while i < n:
        code = frame[i]
        if code == 0 or i + code > n:
            return None
        out += frame[i + 1 : i + code]
        i += code
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def cobs_encode(data: bytes) -> bytes:
    """Canonical COBS, byte-identical to firmware/include/cobs.h (no empty block after a full run)."""
    out, block, full = bytearray(), bytearray(), False
    for b in data:
        full = False
        if b == 0:
            out += bytes([len(block) + 1]) + block
            block = bytearray()
        else:
            block.append(b)
            if len(block) == 254:
                out += b"\xff" + block
                block, full = bytearray(), True
    if not full:
        out += bytes([len(block) + 1]) + block
    return bytes(out)


def packet_ok(buf: bytes) -> int:
    """Returns the packet type if magic, version and CRC are valid, else 0."""
    if len(buf) < HDR.size + 2:
        return 0
    magic, ver, ptype, _, _ = HDR.unpack_from(buf)
    if magic != MAGIC or ver != PROTO_VERSION:
        return 0
    if crc16_ccitt(buf[:-2]) != struct.unpack_from("<H", buf, len(buf) - 2)[0]:
        return 0
    return ptype


@dataclass
class GloveLog:
    side: str
    seq: list = field(default_factory=list)
    t_us: list = field(default_factory=list)
    rx_t_us: list = field(default_factory=list)
    sweep_us: list = field(default_factory=list)
    status: list = field(default_factory=list)
    sensor_ok: list = field(default_factory=list)
    mag: list = field(default_factory=list)
    imu: list = field(default_factory=list)
    imu_temp: list = field(default_factory=list)
    rssi: list = field(default_factory=list)
    events: list = field(default_factory=list)
    lost: int = 0
    ring_drops: int = 0
    tx_fail: int = 0
    batt: tuple = (255, 0)
    last_seq: int | None = None

    def add(self, f: tuple, rx_t_us: int, rssi: int) -> None:
        seq, t_us, sweep, ok, _sat, status, soc, mv, ring, txf = f[5:15]
        if self.last_seq is not None and seq > self.last_seq + 1:
            self.lost += seq - self.last_seq - 1
        self.last_seq = seq
        self.seq.append(seq)
        self.t_us.append(t_us)
        self.rx_t_us.append(rx_t_us)
        self.sweep_us.append(sweep)
        self.status.append(status)
        self.sensor_ok.append(ok)
        self.mag.append(f[15:60])
        self.imu.append(f[60:72])
        self.imu_temp.append(f[72:74])
        self.rssi.append(rssi)
        self.ring_drops, self.tx_fail, self.batt = ring, txf, (soc, mv)

    def arrays(self) -> dict:
        return {
            "seq": np.asarray(self.seq, np.uint32),
            "t_us": np.asarray(self.t_us, np.uint64),
            "rx_t_us": np.asarray(self.rx_t_us, np.uint64),
            "sweep_us": np.asarray(self.sweep_us, np.uint16),
            "status": np.asarray(self.status, np.uint16),
            "sensor_ok": np.asarray(self.sensor_ok, np.uint32),
            "mag": np.asarray(self.mag, np.int16).reshape(-1, N_MAG, 3),
            "imu": np.asarray(self.imu, np.int16).reshape(-1, N_IMU, 6),
            "imu_temp": np.asarray(self.imu_temp, np.int16).reshape(-1, N_IMU),
            "rssi": np.asarray(self.rssi, np.int8),
        }


def clock_fit(t_glove: np.ndarray, t_rx: np.ndarray, bin_s: float = 1.0):
    """Lower-envelope fit of d = rx - glove vs glove time.

    Returns (offset_us at the last sample, drift_ppm, min_latency_spread_us) or None. drift_ppm > 0
    means the glove clock runs FAST relative to the dongle clock.
    """
    if len(t_glove) < 20:
        return None
    tg = t_glove.astype(np.float64)
    d = t_rx.astype(np.float64) - tg
    bins = np.floor((tg - tg[0]) / (bin_s * 1e6)).astype(np.int64)
    xs, ys = [], []
    for b in np.unique(bins):
        idx = np.flatnonzero(bins == b)
        k = idx[np.argmin(d[idx])]
        xs.append(tg[k])
        ys.append(d[k])
    if len(xs) < 2:
        return None
    xs, ys = np.asarray(xs), np.asarray(ys)
    slope, icpt = np.polyfit(xs - xs[0], ys, 1)
    # Refine: refit on the samples closest to the envelope (lowest 2 % of residuals), which uses
    # many more low-latency samples than one minimum per bin.
    r_all = d - (icpt + slope * (tg - xs[0]))
    sel = r_all <= np.quantile(r_all, 0.02)
    if sel.sum() >= 10:
        slope, icpt = np.polyfit(tg[sel] - xs[0], d[sel], 1)
    # rx = glove + d; d(t) = icpt + slope * t  ->  glove advances (1 + drift) per dongle unit
    drift_ppm = -slope / (1.0 + slope) * 1e6
    offset = icpt + slope * (tg[-1] - xs[0])
    resid = ys - (icpt + slope * (xs - xs[0]))
    return offset, drift_ppm, float(np.ptp(resid))


class Reader:
    def __init__(self) -> None:
        self.gloves = {"L": GloveLog("L"), "R": GloveLog("R")}
        self.bad = 0
        self.dongle_status = None
        self.texts: list[str] = []
        self.acks: list[tuple] = []

    def feed_frame(self, raw: bytes) -> None:
        buf = cobs_decode(raw)
        if buf is None or len(buf) < USB_HDR.size:
            self.bad += 1
            return
        kind, side_b, rx_t, rssi, ln = USB_HDR.unpack_from(buf)
        payload = buf[USB_HDR.size : USB_HDR.size + ln]
        if len(payload) != ln:
            self.bad += 1
            return
        side = chr(side_b)
        if kind == K_TEXT:
            self.texts.append(payload.decode("utf-8", "replace"))
            print(f"[text] {self.texts[-1]}")
            return
        if kind == K_STATUS and ln == STATUS.size:
            self.dongle_status = STATUS.unpack(payload)
            return
        ptype = packet_ok(payload)
        if kind != K_GLOVE or ptype == 0 or side not in self.gloves:
            self.bad += 1
            return
        g = self.gloves[side]
        if ptype == T_DATA and ln == DATA.size:
            g.add(DATA.unpack(payload), rx_t, rssi)
        elif ptype == T_EVENT and ln == EVENT.size:
            f = EVENT.unpack(payload)
            g.events.append((EVENTS.get(f[7], f[7]), f[5], f[6], f[8], f[9], rx_t))
            print(f"[{side}] event {g.events[-1][0]} seq={f[5]} t_us={f[6]} arg={f[8]} arg2={f[9]}")
        elif ptype == T_CMDACK and ln == ACK.size:
            f = ACK.unpack(payload)
            self.acks.append((side, f, rx_t))
            print(f"[{side}] ack cmd={f[5]} id={f[6]} result={f[7]} streaming={f[8]} seq={f[9]} t_us={f[10]}")
        else:
            self.bad += 1

    def report(self, window_s: float) -> None:
        for s, g in self.gloves.items():
            n = len(g.seq)
            if n == 0:
                continue
            t = np.asarray(g.t_us, np.float64)
            recent = t > t[-1] - window_s * 1e6
            rate = (recent.sum() - 1) / window_s if recent.sum() > 1 else 0.0
            fit = clock_fit(np.asarray(g.t_us, np.uint64), np.asarray(g.rx_t_us, np.uint64))
            fit_s = (
                f"offset {fit[0] / 1e3:+.3f} ms  drift {fit[1]:+.1f} ppm  envelope spread {fit[2]:.0f} us"
                if fit
                else "clock fit: need more data"
            )
            st = g.status[-1]
            flags = ",".join(name for i, name in enumerate(STATUS_BITS) if st & (1 << i)) or "-"
            print(
                f"[{s}] {n} frames  {rate:6.1f} Hz  lost {g.lost}  ring_drops {g.ring_drops}  tx_fail {g.tx_fail}  "
                f"rssi {np.mean(g.rssi[-200:]):.0f} dBm  sweep {np.mean(g.sweep_us[-200:]):.0f} us  "
                f"batt {g.batt[0]}% {g.batt[1]} mV  ok 0x{g.sensor_ok[-1]:05X}  flags {flags}"
            )
            print(f"    {fit_s}")
        if self.dongle_status:
            d = self.dongle_status
            print(
                f"[dongle] up {d[0] / 1e3:.0f} s  rx L {d[1]} R {d[2]}  bad {d[3]}  queue_drops {d[4]}  "
                f"usb_drops {d[5]}  paired L {d[6]} R {d[7]}  ch {d[8]}"
            )
        if self.bad:
            print(f"[host] bad frames: {self.bad}")


def make_command(spec: str) -> bytes:
    """'L:sync:5', '*:start', 'R:rate:100', 'ping' (= both)."""
    parts = spec.split(":")
    target = "*"
    if parts[0] in ("L", "R", "*"):
        target, parts = parts[0], parts[1:]
    name = parts[0].lower()
    arg = int(parts[1]) if len(parts) > 1 else 0
    if name not in COMMANDS:
        raise ValueError(f"unknown command {name!r}; one of {sorted(COMMANDS)}")
    payload = struct.pack("<BBBH", ord("C"), ord(target), COMMANDS[name], arg)
    return cobs_encode(payload) + b"\x00"


def iter_frames(read_chunk):
    buf = bytearray()
    while True:
        chunk = read_chunk()
        if chunk is None:
            return
        buf += chunk
        while True:
            i = buf.find(b"\x00")
            if i < 0:
                break
            frame = bytes(buf[:i])
            del buf[: i + 1]
            if frame:
                yield frame


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port, or 'auto'")
    src.add_argument("--file", help="decode a captured binary stream instead")
    ap.add_argument("--send", action="append", default=[], help="command to send after opening (repeatable)")
    ap.add_argument("--interval", type=float, default=2.0, help="report period, s")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (0 = forever)")
    ap.add_argument("--save", help="write numpy arrays to this .npz on exit")
    ap.add_argument("--check", action="store_true", help="--file self-test against test_protocol's stream")
    args = ap.parse_args()

    rd = Reader()
    if args.file:
        with open(args.file, "rb") as f:
            data = f.read()
        chunks = iter([data[i : i + 4096] for i in range(0, len(data), 4096)])
        for frame in iter_frames(lambda: next(chunks, None)):
            rd.feed_frame(frame)
        rd.report(window_s=5.0)
        rc = self_check(rd) if args.check else 0
    else:
        import serial  # pyserial
        from serial.tools import list_ports

        port = args.port
        if port == "auto":
            cands = [
                p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x303A, 0x1001)
            ]  # Espressif USB JTAG/serial
            if not cands:
                print("no ESP32-S3 USB serial port found", file=sys.stderr)
                return 2
            port = cands[0]
        ser = serial.Serial(port, 921600, timeout=0.05)
        for spec in args.send:
            ser.write(make_command(spec))
        t0 = last = time.monotonic()
        try:
            for frame in iter_frames(lambda: ser.read(4096)):
                rd.feed_frame(frame)
                now = time.monotonic()
                if now - last >= args.interval:
                    last = now
                    rd.report(window_s=args.interval)
                if args.duration and now - t0 >= args.duration:
                    break
        except KeyboardInterrupt:
            pass
        rc = 0
    if args.save:
        out = {f"{s}_{k}": v for s, g in rd.gloves.items() if g.seq for k, v in g.arrays().items()}
        np.savez_compressed(args.save, **out)
        print(f"saved {args.save}")
    return rc


def self_check(rd: Reader) -> int:
    """Expectations for the stream written by test/host/test_protocol.cpp."""
    L, R = rd.gloves["L"], rd.gloves["R"]
    fl = clock_fit(np.asarray(L.t_us, np.uint64), np.asarray(L.rx_t_us, np.uint64))
    fr = clock_fit(np.asarray(R.t_us, np.uint64), np.asarray(R.rx_t_us, np.uint64))
    a = L.arrays()
    checks = {
        "L frames 1990": len(L.seq) == 1990,
        "R frames 2000": len(R.seq) == 2000,
        "L lost 10": L.lost == 10,
        "R lost 0": R.lost == 0,
        "1 bad frame (junk)": rd.bad == 1,
        "SYNC event": any(e[0] == "SYNC_EDGE" and e[3] == 9 for e in L.events),
        "text frame": rd.texts == ["hello from the test"],
        "dongle status": rd.dongle_status is not None and rd.dongle_status[0] == 6000,
        "mag shape": a["mag"].shape == (1990, 15, 3),
        "mag value": int(a["mag"][0, 2, 1]) == -(3 * 100) + 1 % 7,
        "imu 1 g": int(a["imu"][0, 0, 2]) == 8192,
        "L drift +50 ppm": fl is not None and abs(fl[1] - 50.0) < 2.0,
        "R drift -30 ppm": fr is not None and abs(fr[1] + 30.0) < 2.0,
        # d = rx - glove = -(offset_glove) - drift * t + latency; min latency 800 us
        "L offset ~ +400 ms": fl is not None and abs(fl[0] - (400000 - 50e-6 * 11e6 + 800)) < 300,
    }
    for name, ok in checks.items():
        print(f"  {'ok  ' if ok else 'FAIL'} {name}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
