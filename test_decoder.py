#!/usr/bin/env python3
"""Self-tests for ble_decode.py. Run: python3 test_decoder.py"""
import os
import tempfile
import numpy as np
from ble_decode import (whitening_sequence, ble_crc, decode_capture, load_iq,
                        bits_to_bytes_lsb, crc_to_air_bits)
from make_test_signal import build_pdu, make_capture, to_ci8


def whitening_ubertooth_style(channel, n):
    """Independent byte-oriented formulation (as used in Ubertooth / TI code)."""
    lfsr = int(f"{channel:08b}"[::-1], 2) | 2
    seq = []
    for _ in range(n):
        if lfsr & 0x80:
            lfsr ^= 0x11
            seq.append(1)
        else:
            seq.append(0)
        lfsr = (lfsr << 1) & 0xFF
    return seq


def check(name, cond):
    print(f"{'PASS' if cond else 'FAIL'}  {name}")
    return cond


ok = True
for ch in (37, 38, 39, 0, 17):
    ok &= check(f"whitening ch{ch} matches independent LFSR",
                whitening_sequence(ch, 400) == whitening_ubertooth_style(ch, 400))

# CRC vs scapy's BTLE implementation (independent code)
from scapy.layers.bluetooth4LE import BTLE
for pdu in (build_pdu("AIT"), build_pdu("TEST-DEVICE-1234"), bytes.fromhex("42060102030405 06".replace(" ", ""))):
    ref = BTLE.compute_crc(pdu)            # 3 bytes, as sent on air (each byte LSB first)
    ours = bits_to_bytes_lsb(crc_to_air_bits(ble_crc(pdu)))
    ok &= check(f"CRC matches scapy for {pdu.hex()[:16]}...", ref == ours)

fs80, fc_all = 80e6, 2441e6

def run(label, iq, fs, fc, expect_name, expect_ch, n_expect=1):
    pk = decode_capture(iq, fs, fc)
    good = [p for p in pk if p["status"] == "CRC OK"]
    names = [dict(p.get("ad", [])).get("Name") for p in good]
    chans = [p["channel"] for p in good]
    return check(f"{label}: got {len(good)} CRC-OK {names} on {chans}",
                 len(good) == n_expect and expect_name in names and expect_ch in chans)

# 1. Short packet in a 205 us snapshot at 80 MS/s, centered between channels
iq = make_capture([(build_pdu("AIT"), 38)], fs80, fc_all, 205e-6, snr_db=20, starts=[300])
ok &= run("80 MS/s, ch38, 205 us", iq, fs80, fc_all, "AIT", 38)

# 2. All three advertising channels at once (edges of the 80 MHz span)
for ch in (37, 39):
    iq = make_capture([(build_pdu("EDGE"), ch)], fs80, fc_all, 205e-6, snr_db=20, starts=[300])
    ok &= run(f"80 MS/s, ch{ch} near span edge", iq, fs80, fc_all, "EDGE", ch)

# 3. Spectrum flipped (I/Q swapped), as some radios deliver it
iq = make_capture([(build_pdu("FLIP"), 38)], fs80, fc_all, 205e-6, snr_db=20, starts=[300], flip=True)
ok &= run("flipped IQ", iq, fs80, fc_all, "FLIP", 38)

# 4. Full-length packet at 20 MS/s, longer capture, 8-bit samples written to disk
long_pdu = build_pdu("ESP32-BLE-DEMO", mfg=bytes.fromhex("e502deadbeef0102"))
iq = make_capture([(long_pdu, 38)], 20e6, 2426e6, 800e-6, snr_db=15, cfo=-120e3, starts=[1500])
path = os.path.join(tempfile.mkdtemp(), "t.ci8")
to_ci8(iq, path)
ok &= run("20 MS/s, ci8 file, long packet, -120 kHz offset", load_iq(path), 20e6, 2426e6, "ESP32-BLE-DEMO", 38)

# 5. Truncated packet: starts too late to finish inside the snapshot
iq = make_capture([(long_pdu, 38)], fs80, fc_all, 205e-6, snr_db=20, starts=[4000])
pk = decode_capture(iq, fs80, fc_all)
ok &= check(f"truncated packet flagged: {[p['status'] for p in pk]}",
            len(pk) == 1 and pk[0]["status"].startswith("truncated"))

# 6. Noise only: no false packets
rng = np.random.default_rng(7)
noise = (rng.standard_normal(16400) + 1j * rng.standard_normal(16400)).astype(np.complex64)
pk = decode_capture(noise, fs80, fc_all)
ok &= check(f"noise only: {sum(p['status']=='CRC OK' for p in pk)} false CRC-OK packets",
            not any(p["status"] == "CRC OK" for p in pk))

# 7. SNR sweep
print("\nSNR sweep (50 trials each, 80 MS/s, ch38):")
for snr in (6, 8, 10, 12, 15):
    hits = 0
    for t in range(50):
        iq = make_capture([(build_pdu("AIT"), 38)], fs80, fc_all, 205e-6, snr_db=snr, seed=100 + t, starts=[200 + 7 * t])
        hits += any(p["status"] == "CRC OK" for p in decode_capture(iq, fs80, fc_all))
    print(f"  {snr:>2} dB: {hits}/50 decoded")

print("\nALL PASS" if ok else "\nSOME TESTS FAILED")
