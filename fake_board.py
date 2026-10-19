#!/usr/bin/env python3
"""
fake_board.py - Pretends to be an ESP32-S3 running ESP-SDR on a virtual serial
port, so esp_capture.py can be tested without hardware. Linux/macOS only.

  python3 fake_board.py          # prints the port name, e.g. /dev/pts/5
  python3 esp_capture.py --port /dev/pts/5 --count 20
"""
import os
import pty
import random
import sys
import tty
import zlib

import numpy as np
from make_test_signal import build_pdu, make_capture


def run(hit_rate=0.3, max_samples=16380, flip=False):
    master, slave = pty.openpty()
    tty.setraw(slave)
    print(os.ttyname(slave), flush=True)
    os.write(master, b"ESP-ROM:esp32s3-20210327\r\nboot noise \x00\xff\r\n")   # like a reset
    fc, buf, n = 2426, b"", 0
    rates = {0: 80e6, 1: 40e6, 6: 16e6}
    while True:
        try:
            buf += os.read(master, 4096)
        except OSError:
            return
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            words = line.decode(errors="replace").strip().split()
            if not words:
                continue
            c = words[0]
            out = None
            if c == "SYNC":
                out = line.strip()
            elif c == "RELEASE":
                out = b"OK"
            elif c == "INFO":
                out = f"S3SDR 6 burst {max_samples}".encode()
            elif c == "LIMITS?":
                out = b'LIMITS {"gain":[0,82,1],"bandwidth":[2,40,1,20],"rates":[80,40,16],"bits":[8,10]}'
            elif c == "FREQ":
                fc = int(words[1])
                out = b"OK"
            elif c in ("GAIN", "BANDWIDTH"):
                out = b"OK"
            elif c == "CAP16":
                samples, code = int(words[1]), int(words[2])
                if code not in rates:
                    out = b"ERR rate"
                else:
                    fs = rates[code]
                    n += 1
                    pdus = []
                    if random.random() < hit_rate:
                        pdus = [(build_pdu(random.choice(["AIT", "Phone", "Beacon-7"]),
                                           mfg=bytes.fromhex("4c000215aabbccdd")), 38)]
                    dur = samples / fs
                    iq = make_capture(pdus, fs, fc * 1e6, dur, snr_db=18, seed=n,
                                      starts=[int(random.uniform(0, 0.4) * samples)] if pdus else None,
                                      flip=flip) if pdus else \
                        make_capture([], fs, fc * 1e6, dur, seed=n)
                    iq = iq[:samples] * (60 / max(np.abs(iq).max(), 1e-9))
                    raw = np.empty(2 * len(iq), np.int8)
                    raw[0::2] = np.clip(np.round(iq.real), -127, 127)
                    raw[1::2] = np.clip(np.round(iq.imag), -127, 127)
                    raw = raw.tobytes()
                    os.write(master, f"DATA {len(iq)} {zlib.crc32(raw):08x} {int(dur*1e6)}\n".encode() + raw)
                    continue
            else:
                out = b"ERR unknown"
            os.write(master, out + b"\n")


if __name__ == "__main__":
    run(flip="--flip" in sys.argv)
