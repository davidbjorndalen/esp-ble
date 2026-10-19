#!/usr/bin/env python3
"""
esp_capture.py - Grab raw IQ snapshots from an ESP32 running ESP-SDR firmware,
save them to disk, and (optionally) decode BLE advertising packets live.

Talks the firmware's serial protocol directly (see esp-sdr docs/protocol.md):
  FREQ <MHz>, GAIN ..., BANDWIDTH ..., CAP16 <samples> <rate-code>
Each CAP16 reply is "DATA <samples> <crc32> <us>" followed by signed 8-bit
I,Q pairs, which is exactly the .ci8 format ble_decode.py reads.

Close the browser viewer first: only one program can hold the serial port.

  python3 esp_capture.py                        # 16 MS/s at 2426 MHz, decode live
  python3 esp_capture.py --count 200 --save caps
  python3 esp_capture.py --port /dev/ttyACM0 --gain 40
"""
import argparse
import glob
import os
import sys
import time
import zlib

import numpy as np

RATE_CODES = {80: 0, 40: 1, 20: 2, 10: 3, 8: 4, 4: 5, 16: 6, 32: 7, 10.667: 8, 6.4: 9}


class ESPSDR:
    def __init__(self, port, baud=2_000_000):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.2)

    def close(self):
        try:
            self.cmd("RELEASE")
        except Exception:
            pass
        self.s.close()

    def sync(self, seconds=10):
        """Opening the port can reset the board; skip boot noise until our marker echoes."""
        deadline = time.monotonic() + seconds
        self.s.timeout = 0.2
        while time.monotonic() < deadline:
            marker = f"SYNC {time.monotonic_ns()}".encode()
            self.s.write(b"\nRELEASE\n" + marker + b"\n")
            end = time.monotonic() + 1
            while time.monotonic() < end:
                if self.s.readline().strip() == marker:
                    self.s.timeout = 5
                    return
        raise TimeoutError("No reply from the board. Is the browser viewer still connected?")

    def cmd(self, text):
        self.s.write((text + "\n").encode("ascii"))
        reply = self.s.readline().decode("ascii", "replace").strip()
        if not reply:
            raise TimeoutError(f"No reply to {text!r}")
        return reply

    def check(self, text):
        reply = self.cmd(text)
        if reply.startswith("ERR"):
            raise RuntimeError(f"{text!r} -> {reply}")
        return reply

    def read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.s.read(n - len(buf))
            if not chunk:
                raise TimeoutError(f"Capture data stopped after {len(buf)}/{n} bytes")
            buf.extend(chunk)
        return bytes(buf)

    def capture(self, samples, rate_code):
        reply = self.check(f"CAP16 {samples} {rate_code}")
        parts = reply.split()
        if parts[0] != "DATA":
            raise RuntimeError(f"Unexpected reply: {reply}")
        n, crc, us = int(parts[1]), int(parts[2], 16), parts[3] if len(parts) > 3 else "?"
        raw = self.read_exact(2 * n)
        if zlib.crc32(raw) != crc:
            raise IOError("CRC mismatch")
        return raw, us


def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
                   + glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        sys.exit("No serial port found. Plug in the board or pass --port.")
    return ports[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/ttyACM* or /dev/ttyUSB*)")
    ap.add_argument("--fc", type=int, default=2426, help="center frequency, whole MHz (2426 = BLE ch38)")
    ap.add_argument("--rate", type=float, default=16, help="sample rate MS/s (S3: 80, 40, 16)")
    ap.add_argument("--samples", type=int, default=0, help="samples per snapshot (0 = board maximum)")
    ap.add_argument("--gain", default="hw", help="'hw' for hardware AGC, or a manual gain index")
    ap.add_argument("--bw", type=float, help="analog bandwidth MHz (0 = widest)")
    ap.add_argument("--count", type=int, default=0, help="number of snapshots (0 = until Ctrl+C)")
    ap.add_argument("--save", metavar="DIR", help="save every snapshot as .ci8 in DIR")
    ap.add_argument("--save-hits", metavar="DIR", help="save only snapshots with a decoded packet")
    ap.add_argument("--no-decode", action="store_true", help="just capture, don't decode")
    ap.add_argument("--quiet", action="store_true", help="skip per-packet output; show the device table")
    ap.add_argument("--table-every", type=int, default=200, help="print the device table every N snapshots")
    ap.add_argument("--baud", type=int, default=2_000_000)
    a = ap.parse_args()

    if a.rate not in RATE_CODES:
        sys.exit(f"Unknown rate {a.rate}; choose from {sorted(RATE_CODES)}")
    decode = None
    if not a.no_decode:
        from ble_decode import decode_capture, print_packet, DeviceTracker
        decode = decode_capture
        tracker = DeviceTracker()

    port = a.port or find_port()
    print(f"Opening {port} ...")
    dev = ESPSDR(port, a.baud)
    try:
        dev.sync()
        info = dev.check("INFO")
        max_samples = int(info.split()[-1])
        print(f"Board: {info}")
        limits = dev.cmd("LIMITS?")
        if f"{a.rate:g}" not in limits and str(a.rate) not in limits:
            print(f"Warning: {a.rate:g} MS/s may not be supported here: {limits}")
        samples = min(a.samples or max_samples, max_samples)
        dev.check(f"FREQ {a.fc}")
        dev.check("GAIN HARDWARE" if a.gain == "hw" else f"GAIN MANUAL {int(a.gain)}")
        if a.bw is not None:
            dev.check(f"BANDWIDTH {a.bw:g}")
        fs = a.rate * 1e6
        print(f"Capturing {samples} samples at {a.rate:g} MS/s, {a.fc} MHz "
              f"= {samples / fs * 1e6:.0f} us per snapshot. Ctrl+C to stop.\n")
        for d in (a.save, a.save_hits):
            if d:
                os.makedirs(d, exist_ok=True)

        n = hits = errors = 0
        t0 = time.monotonic()
        while not a.count or n < a.count:
            try:
                raw, us = dev.capture(samples, RATE_CODES[a.rate])
            except (IOError, TimeoutError, RuntimeError) as e:
                errors += 1
                print(f"  capture error: {e}; resyncing")
                dev.sync()
                continue
            n += 1
            name = f"cap_{time.strftime('%Y%m%d-%H%M%S')}_{n:05d}_{a.fc}MHz_{a.rate:g}Msps.ci8"
            if a.save:
                with open(os.path.join(a.save, name), "wb") as f:
                    f.write(raw)
            if decode:
                iq8 = np.frombuffer(raw, np.int8).astype(np.float32)
                iq = (iq8[0::2] + 1j * iq8[1::2]).astype(np.complex64)
                good = [p for p in decode(iq, fs, a.fc * 1e6) if p["status"] == "CRC OK"]
                for p in good:
                    tracker.add(p, n)
                    if not a.quiet:
                        print_packet(p, f"#{n} ")
                if good:
                    hits += 1
                    if a.save_hits:
                        with open(os.path.join(a.save_hits, name), "wb") as f:
                            f.write(raw)
            if decode and a.table_every and n % a.table_every == 0:
                print("\n" + tracker.table() + "\n")
            elif n % 50 == 0:
                rate = n / (time.monotonic() - t0)
                print(f"  ... {n} snapshots ({rate:.1f}/s), {hits} with packets, {errors} errors")
    except KeyboardInterrupt:
        pass
    finally:
        dev.close()
    print(f"\nDone: {n} snapshots, {hits} with decoded packets, {errors} errors\n")
    if decode:
        print(tracker.table())


if __name__ == "__main__":
    main()
