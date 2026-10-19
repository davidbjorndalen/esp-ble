#!/usr/bin/env python3
"""
make_test_signal.py - Synthesize BLE advertising packets as raw IQ, the way
the ESP32 would capture them. Use it to check the decoder before real captures.

  python3 make_test_signal.py test.cf32 --fs 80 --fc 2441 --channel 38 --name AIT
"""
import argparse
import numpy as np
from scipy import signal
from ble_decode import (ADV_CHANNELS, SYNC_BITS, ble_crc, crc_to_air_bits,
                        bytes_to_bits_lsb, whitening_sequence)

CHANNEL_INDEX = {37: 37, 38: 38, 39: 39}


def build_pdu(name, mac=bytes.fromhex("C0FFEE123456"), pdu_type=2, mfg=b""):
    ad = bytes([2, 0x01, 0x06])
    if name:
        nb = name.encode()
        ad += bytes([len(nb) + 1, 0x09]) + nb
    if mfg:
        ad += bytes([len(mfg) + 1, 0xFF]) + mfg
    payload = mac[::-1] + ad                                  # address goes LSB-byte first
    header = bytes([pdu_type | 0x40, len(payload)])           # 0x40 = random address
    return header + payload


def packet_bits(pdu, channel):
    body = bytes_to_bits_lsb(pdu) + crc_to_air_bits(ble_crc(pdu))
    white = whitening_sequence(channel, len(body))
    return SYNC_BITS + [b ^ w for b, w in zip(body, white)]


def gfsk(bits, fs, bt=0.5, dev=250e3):
    sps = fs / 1e6
    n = int(len(bits) * sps)
    t = np.arange(n)
    nrz = (2 * np.array(bits) - 1.0)[np.minimum((t / sps).astype(int), len(bits) - 1)]
    # Gaussian pulse shaping, BT = 0.5
    span = int(3 * sps)
    tt = np.arange(-span, span + 1) / sps
    sigma = np.sqrt(np.log(2)) / (2 * np.pi * bt)
    g = np.exp(-tt ** 2 / (2 * sigma ** 2))
    g /= g.sum()
    f = np.convolve(nrz, g, mode="same") * dev
    return np.exp(2j * np.pi * np.cumsum(f) / fs)


def make_capture(pdus, fs, fc, duration, snr_db=20, cfo=40e3, starts=None,
                 flip=False, seed=1):
    """pdus: list of (pdu_bytes, channel). Returns complex64 IQ."""
    rng = np.random.default_rng(seed)
    n = int(duration * fs)
    iq = np.zeros(n, complex)
    for i, (pdu, ch) in enumerate(pdus):
        burst = gfsk(packet_bits(pdu, ch), fs)
        s = starts[i] if starts else rng.integers(0, max(1, n - len(burst)))
        s = int(s)
        burst = burst[:n - s]
        k = np.arange(s, s + len(burst))
        off = ADV_CHANNELS[ch] - fc + cfo
        iq[s:s + len(burst)] += burst * np.exp(2j * np.pi * off * k / fs + 1j * rng.uniform(0, 6.28))
    # Noise is set relative to the signal power inside a ~2 MHz channel.
    noise_power_per_hz = 1.0 / (10 ** (snr_db / 10)) / 2e6
    sigma = np.sqrt(noise_power_per_hz * fs / 2)
    iq += sigma * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    if flip:
        iq = np.conj(iq)
    return iq.astype(np.complex64)


def to_ci8(iq, path):
    scale = 100 / np.max(np.abs(np.concatenate([iq.real, iq.imag])))
    inter = np.empty(2 * len(iq), np.int8)
    inter[0::2] = np.clip(np.round(iq.real * scale), -127, 127)
    inter[1::2] = np.clip(np.round(iq.imag * scale), -127, 127)
    inter.tofile(path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--fs", type=float, default=80)
    ap.add_argument("--fc", type=float, default=2441)
    ap.add_argument("--channel", type=int, default=38)
    ap.add_argument("--name", default="AIT")
    ap.add_argument("--duration_us", type=float, default=205)
    ap.add_argument("--snr", type=float, default=20)
    ap.add_argument("--ci8", action="store_true")
    ap.add_argument("--start_us", type=float, default=3, help="where the packet begins")
    a = ap.parse_args()
    iq = make_capture([(build_pdu(a.name), a.channel)], a.fs * 1e6, a.fc * 1e6,
                      a.duration_us * 1e-6, a.snr, starts=[a.start_us * a.fs])
    to_ci8(iq, a.out) if a.ci8 else iq.tofile(a.out)
    print(f"wrote {len(iq)} samples to {a.out}")
