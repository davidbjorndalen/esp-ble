#!/usr/bin/env python3
"""
ble_decode.py - Decode Bluetooth Low Energy advertising packets from raw IQ.

Built for ESP32 raw-IQ captures (ESP-SDR / ESP-WebSDR), but works with any
complex baseband recording of the 2.4 GHz band.

Pipeline, per advertising channel that falls inside the capture:
  1. Mix the channel down to 0 Hz and low-pass filter it (~1 MHz wide).
  2. Decimate to a handful of samples per bit.
  3. FM-demodulate (phase difference between samples = instantaneous frequency).
  4. Slide the known preamble + access-address pattern along and look for matches.
  5. At each match: estimate carrier offset, slice bits, undo whitening,
     check the CRC, and parse the advertising data.

Usage:
  python3 ble_decode.py capture.cf32 --fs 80 --fc 2441
  python3 ble_decode.py snaps/*.ci8 --fs 20 --fc 2426 --format ci8
"""
import argparse
import os
import sys
import numpy as np
from scipy import signal

# --- BLE constants --------------------------------------------------------
ADV_ACCESS_ADDRESS = 0x8E89BED6
ADV_CRC_INIT = 0x555555
BIT_RATE = 1e6                       # LE 1M PHY
ADV_CHANNELS = {37: 2402e6, 38: 2426e6, 39: 2480e6}

PDU_TYPES = {
    0: "ADV_IND", 1: "ADV_DIRECT_IND", 2: "ADV_NONCONN_IND", 3: "SCAN_REQ",
    4: "SCAN_RSP", 5: "CONNECT_IND", 6: "ADV_SCAN_IND", 7: "ADV_EXT_IND",
}
COMPANY_IDS = {
    0x004C: "Apple", 0x0006: "Microsoft", 0x0075: "Samsung", 0x00E0: "Google",
    0x0059: "Nordic Semiconductor", 0x02E5: "Espressif", 0x0087: "Garmin",
    0x0157: "Huami", 0x038F: "Xiaomi", 0x0499: "Ruuvi Innovations",
}
# 16-bit service UUIDs assigned to member companies by the Bluetooth SIG.
UUID16_NAMES = {
    0xFE00: "Amazon", 0xFE03: "Amazon", 0xFE0F: "Signify (Philips Hue)",
    0xFEAF: "Nest Labs", 0xFE9F: "Google", 0xFE2C: "Google Fast Pair",
    0xFEAA: "Google Eddystone", 0xFD6F: "Exposure Notification", 0xFEED: "Tile",
    0xFE07: "Sonos", 0x180F: "Battery", 0x180D: "Heart Rate", 0x181A: "Environmental Sensing",
}
# First byte of each message inside Apple manufacturer data.
APPLE_TYPES = {
    0x02: "iBeacon", 0x05: "AirDrop", 0x07: "AirPods/Beats pairing", 0x09: "AirPlay target",
    0x0C: "Handoff", 0x0F: "Nearby Action", 0x10: "Nearby Info", 0x12: "Find My",
}


def uuid16_str(u):
    return f"0x{u:04X}" + (f" {UUID16_NAMES[u]}" if u in UUID16_NAMES else "")


AD_TYPES = {
    0x01: "Flags", 0x02: "UUID16 (partial)", 0x03: "UUID16", 0x06: "UUID128 (partial)",
    0x07: "UUID128", 0x08: "Short name", 0x09: "Name", 0x0A: "TX power",
    0x16: "Service data", 0x19: "Appearance", 0xFF: "Manufacturer data",
}


def int_to_bits_lsb(value, nbits):
    """BLE sends every field least-significant bit first."""
    return [(value >> i) & 1 for i in range(nbits)]


def bits_to_bytes_lsb(bits):
    out = bytearray()
    for i in range(0, len(bits) - len(bits) % 8, 8):
        out.append(sum(b << k for k, b in enumerate(bits[i:i + 8])))
    return bytes(out)


def bytes_to_bits_lsb(data):
    return [(byte >> k) & 1 for byte in data for k in range(8)]


def whitening_sequence(channel, n):
    """Whitening LFSR (x^7 + x^4 + 1), seeded from the channel index.
    Position 0 = 1, positions 1..6 = channel index MSB..LSB; output from position 6."""
    reg = [1] + [(channel >> (5 - i)) & 1 for i in range(6)]
    seq = []
    for _ in range(n):
        out = reg[6]
        seq.append(out)
        reg = [out] + reg[:6]
        reg[4] ^= out
    return seq


def dewhiten(bits, channel):
    return [b ^ w for b, w in zip(bits, whitening_sequence(channel, len(bits)))]


def ble_crc(data, init=ADV_CRC_INIT):
    """24-bit BLE CRC over PDU bytes (bits fed LSB first). Returns the register value."""
    crc = init
    for byte in data:
        for i in range(8):
            fb = ((crc >> 23) & 1) ^ ((byte >> i) & 1)
            crc = (crc << 1) & 0xFFFFFF
            if fb:
                crc ^= 0x00065B
    return crc


def crc_to_air_bits(crc):
    """The CRC register is sent most-significant bit first."""
    return [(crc >> (23 - i)) & 1 for i in range(24)]


SYNC_BITS = [0, 1, 0, 1, 0, 1, 0, 1] + int_to_bits_lsb(ADV_ACCESS_ADDRESS, 32)


# --- IQ loading -----------------------------------------------------------
def load_iq(path, fmt=None):
    ext = os.path.splitext(path)[1].lower().lstrip(".")
    fmt = fmt or {"cfile": "cf32", "cf32": "cf32", "fc32": "cf32", "ci8": "ci8",
                  "cs8": "ci8", "cu8": "cu8", "ci16": "ci16", "cs16": "ci16",
                  "npy": "npy", "wav": "wav"}.get(ext)
    if fmt is None:
        sys.exit(f"Can't guess the format of {path}; pass --format")
    if fmt == "npy":
        return np.load(path).astype(np.complex64).ravel()
    if fmt == "wav":
        from scipy.io import wavfile
        _, d = wavfile.read(path)
        d = d.astype(np.float32)
        return (d[:, 0] + 1j * d[:, 1]).astype(np.complex64)
    dtype = {"cf32": np.float32, "ci8": np.int8, "cu8": np.uint8, "ci16": np.int16}[fmt]
    raw = np.fromfile(path, dtype=dtype).astype(np.float32)
    if fmt == "cu8":
        raw -= 127.5
    return (raw[0::2] + 1j * raw[1::2]).astype(np.complex64)


# --- DSP ------------------------------------------------------------------
def channelize(iq, fs, offset_hz, sps_target=8):
    """Shift one BLE channel to 0 Hz, filter it, and decimate."""
    decim = max(1, int(round(fs / (BIT_RATE * sps_target))))
    fs_out = fs / decim
    n = np.arange(len(iq))
    mixed = iq * np.exp(-2j * np.pi * offset_hz / fs * n).astype(np.complex64)
    taps = signal.firwin(int(8 * decim) | 1, 0.9e6, fs=fs)   # ~1.8 MHz wide passband
    y = signal.upfirdn(taps, mixed, down=decim)
    delay = (len(taps) - 1) // 2 // decim                    # line samples back up in time
    return y[delay:], fs_out


def fm_demod(y, fs_out):
    """Instantaneous frequency in Hz."""
    return np.angle(y[1:] * np.conj(y[:-1])) * fs_out / (2 * np.pi)


def find_syncs(freq, sps, threshold):
    """Normalized correlation of the frequency track against the sync pattern."""
    template = np.repeat(2 * np.array(SYNC_BITS) - 1.0, sps)
    L = len(template)
    if len(freq) < L:
        return []
    x = freq - np.convolve(freq, np.ones(L) / L, mode="same")      # strip carrier offset
    num = np.correlate(x, template, mode="valid")
    energy = np.sqrt(np.convolve(x ** 2, np.ones(L), mode="valid") * L)
    corr = num / np.maximum(energy, 1e-12)
    hits = []
    idx = np.flatnonzero(np.abs(corr) > threshold)
    while idx.size:
        group = idx[idx < idx[0] + L]
        best = group[np.argmax(np.abs(corr[group]))]
        hits.append((int(best), float(corr[best])))
        idx = idx[idx >= idx[0] + L]
    return hits


def slice_bits(freq, start, sps, nbits, offset, invert):
    """Average the middle half of each bit period, compare to the carrier offset."""
    bits = []
    lo, hi = int(sps * 0.25), max(int(sps * 0.75), int(sps * 0.25) + 1)
    for k in range(nbits):
        a = int(round(start + k * sps))
        seg = freq[a + lo:a + hi]
        if len(seg) == 0:
            break
        v = (seg.mean() - offset) * (-1 if invert else 1)
        bits.append(1 if v > 0 else 0)
    return bits


# --- Packet parsing -------------------------------------------------------
def mac_str(b):
    return ":".join(f"{x:02X}" for x in reversed(b))


def parse_ad(data):
    fields, i = [], 0
    while i < len(data):
        ln = data[i]
        if ln == 0 or i + 1 + ln > len(data):
            break
        t, v = data[i + 1], data[i + 2:i + 1 + ln]
        name = AD_TYPES.get(t, f"type 0x{t:02X}")
        if t in (0x08, 0x09):
            val = v.decode("utf-8", "replace")
        elif t == 0xFF and len(v) >= 2:
            cid = v[0] | v[1] << 8
            val = f"{COMPANY_IDS.get(cid, f'company 0x{cid:04X}')}: {v[2:].hex()}"
            if cid == 0x004C and len(v) > 2:
                val += f" ({APPLE_TYPES.get(v[2], f'type 0x{v[2]:02X}')})"
        elif t in (0x02, 0x03):
            val = ", ".join(uuid16_str(v[j] | v[j + 1] << 8) for j in range(0, len(v) - 1, 2))
        elif t == 0x16 and len(v) >= 2:
            val = f"{uuid16_str(v[0] | v[1] << 8)}: {v[2:].hex()}"
        elif t == 0x0A and v:
            val = f"{int.from_bytes(v[:1], 'little', signed=True)} dBm"
        elif t == 0x01 and v:
            val = f"0x{v[0]:02X}"
        else:
            val = v.hex()
        fields.append((name, val))
        i += 1 + ln
    return fields


def gfsk_shape(bits, sps, bt=0.5):
    """Expected frequency track (unit deviation) for these bits after BLE's Gaussian
    filter. Fitting against this shape, rather than square pulses, removes a ~8 kHz bias."""
    nrz = np.repeat(2 * np.asarray(bits, float) - 1.0, sps)
    span = int(3 * sps)
    t = np.arange(-span, span + 1) / sps
    sigma = np.sqrt(np.log(2)) / (2 * np.pi * bt)
    g = np.exp(-t ** 2 / (2 * sigma ** 2))
    return np.convolve(nrz, g / g.sum(), mode="same")


def fit_offset(freq, start, bits, sps, invert):
    """Least-squares fit of freq = dev * shape + offset over a stretch of known bits."""
    shape = gfsk_shape(bits, sps) * (-1 if invert else 1)
    seg = freq[start:start + len(shape)]
    shape = shape[:len(seg)]
    A = np.vstack([shape, np.ones_like(shape)]).T
    (dev, offset), *_ = np.linalg.lstsq(A, seg, rcond=None)
    return dev, offset


def decode_packet(freq, start, sps, channel, corr):
    invert = corr < 0
    sync_n = len(SYNC_BITS)
    # Carrier offset from the known sync pattern; refined over the whole packet below.
    dev, offset = fit_offset(freq, start, SYNC_BITS, sps, invert)

    pdu_start = start + sync_n * sps
    header = dewhiten(slice_bits(freq, pdu_start, sps, 16, offset, invert), channel)
    if len(header) < 16:
        return None
    hdr = bits_to_bytes_lsb(header)
    pdu_type, length = hdr[0] & 0x0F, hdr[1]
    n_total = 16 + 8 * length + 24
    raw = slice_bits(freq, pdu_start, sps, n_total, offset, invert)
    pkt = {"channel": channel, "corr": abs(corr), "cfo_khz": offset / 1e3,
           "dev_khz": abs(dev) / 1e3, "type": PDU_TYPES.get(pdu_type, f"type {pdu_type}"),
           "tx_random": bool(hdr[0] & 0x40), "length": length,
           "start_us": start / sps}
    if len(raw) < n_total:
        pkt["status"] = f"truncated ({len(raw) - 16}/{n_total - 16} bits after header)"
        pkt["partial"] = bits_to_bytes_lsb(dewhiten(raw, channel))
        return pkt
    bits = dewhiten(raw, channel)
    pdu = bits_to_bytes_lsb(bits[:16 + 8 * length])
    crc_ok = crc_to_air_bits(ble_crc(pdu)) == bits[16 + 8 * length:]
    pkt["status"] = "CRC OK" if crc_ok else "CRC FAIL"
    if crc_ok:
        # Every bit is now known, so fit the offset over the full packet: less noise.
        dev, offset = fit_offset(freq, start, SYNC_BITS + raw, sps, invert)
        pkt["cfo_khz"], pkt["dev_khz"] = offset / 1e3, abs(dev) / 1e3
    pkt["pdu"] = pdu
    payload = pdu[2:]
    if pdu_type in (0, 2, 4, 6) and len(payload) >= 6:
        pkt["adv_addr"] = mac_str(payload[:6])
        pkt["ad"] = parse_ad(payload[6:])
    elif pdu_type in (1, 3, 5) and len(payload) >= 12:
        first, second = mac_str(payload[:6]), mac_str(payload[6:12])
        # ADV_DIRECT_IND: AdvA then TargetA.  SCAN_REQ / CONNECT_IND: sender then AdvA.
        pkt["adv_addr"], pkt["other_addr"] = (first, second) if pdu_type == 1 else (second, first)
    return pkt


def decode_capture(iq, fs, fc, threshold=0.6, sps=8, flip="auto"):
    """flip: 'no', 'yes' (I and Q swapped, spectrum mirrored), or 'auto' (try both,
    keep whichever passes CRC). The ESP32's I/Q ordering isn't documented, so auto
    is the safe default until you know which way your board delivers samples."""
    if flip == "yes":
        return _decode_one(np.conj(iq), fs, fc, threshold, sps)
    normal = _decode_one(iq, fs, fc, threshold, sps)
    if flip == "no":
        return normal
    mirrored = [p for p in _decode_one(np.conj(iq), fs, fc, threshold, sps)
                if p["status"] == "CRC OK"]
    # When the channel sits exactly at the center frequency, both passes find it.
    seen = {(p["channel"], round(p["start_us"])) for p in normal if p["status"] == "CRC OK"}
    mirrored = [p for p in mirrored if (p["channel"], round(p["start_us"])) not in seen]
    for p in mirrored:
        p["flipped"] = True
    good_starts = {(round(p["start_us"]), p["channel"]) for p in mirrored}
    # Drop junk from the normal pass that is really a mirrored packet.
    kept = [p for p in normal if p["status"] == "CRC OK"
            or not any(abs(p["start_us"] - s) < 5 for s, _ in good_starts)]
    return sorted(kept + mirrored, key=lambda p: p["start_us"])


def _decode_one(iq, fs, fc, threshold, sps):
    packets = []
    for ch, f in ADV_CHANNELS.items():
        off = f - fc
        if abs(off) + 1e6 > fs / 2:
            continue                                         # channel outside the capture
        y, fs_out = channelize(iq, fs, off, sps)
        sps_eff = fs_out / BIT_RATE
        freq = fm_demod(y, fs_out)
        level = 10 * np.log10(np.mean(np.abs(y) ** 2) + 1e-20)
        found = []
        for start, corr in find_syncs(freq, int(round(sps_eff)), threshold):
            pkt = decode_packet(freq, start, int(round(sps_eff)), ch, corr)
            if pkt:
                end = start + int((40 + 16 + 8 * min(pkt["length"], 255) + 24) * sps_eff)
                found.append((pkt, start, min(end, len(y))))
        # Signal-to-noise: power during each packet vs the parts of the snapshot that hold
        # no packet at all. Works under AGC, since both come from the same snapshot.
        quiet = np.ones(len(y), bool)
        for _, s, e in found:
            quiet[max(0, s - 8):e + 8] = False
        p_noise = np.mean(np.abs(y[quiet]) ** 2) if quiet.sum() > 100 else np.nan
        for pkt, s, e in found:
            p_sig = np.mean(np.abs(y[s:e]) ** 2)
            pkt["power_db"] = 10 * np.log10(p_sig + 1e-20)
            pkt["snr_db"] = 10 * np.log10(max(p_sig - p_noise, 1e-20) / p_noise) \
                if np.isfinite(p_noise) and p_noise > 0 else float("nan")
            pkt["floor_db"] = level
            packets.append(pkt)
    return packets


def print_packet(pkt, name=""):
    head = (f"[ch{pkt['channel']}] {pkt['type']:<16} {pkt['status']:<10} "
            f"len={pkt['length']:<3} offset={pkt['cfo_khz']:+.0f} kHz "
            f"match={pkt['corr']:.2f} t={pkt['start_us']:.1f} us"
            f"{' [IQ flipped]' if pkt.get('flipped') else ''}")
    print(f"{name}{head}")
    if "snr_db" in pkt and np.isfinite(pkt["snr_db"]):
        print(f"    SNR: {pkt['snr_db']:.0f} dB")
    if "adv_addr" in pkt:
        kind = "random" if pkt["tx_random"] else "public"
        if pkt["type"] in ("SCAN_REQ", "CONNECT_IND"):
            who = "scanner" if pkt["type"] == "SCAN_REQ" else "initiator"
            print(f"    from {who}: {pkt['other_addr']} ({kind}) -> to advertiser: {pkt['adv_addr']}")
        elif pkt["type"] == "ADV_DIRECT_IND":
            print(f"    address: {pkt['adv_addr']} ({kind}) -> target: {pkt['other_addr']}")
        else:
            print(f"    address: {pkt['adv_addr']} ({kind})")
    for k, v in pkt.get("ad", []):
        print(f"    {k}: {v}")
    if "pdu" in pkt and pkt["status"] != "CRC OK":
        print(f"    raw: {pkt['pdu'].hex()}")
    if "partial" in pkt:
        print(f"    partial: {pkt['partial'].hex()}")


class DeviceTracker:
    """Groups CRC-checked packets by the address of the device that sent them."""

    def __init__(self):
        self.devices = {}

    def add(self, pkt, snapshot=None):
        if pkt.get("status") != "CRC OK" or "adv_addr" not in pkt:
            return
        sender = pkt["other_addr"] if pkt["type"] in ("SCAN_REQ", "CONNECT_IND") else pkt["adv_addr"]
        d = self.devices.setdefault(sender, {
            "count": 0, "offsets": [], "snrs": [], "types": set(), "name": "",
            "maker": "", "random": pkt["tx_random"], "first": snapshot, "last": snapshot,
            "fchan": ADV_CHANNELS[pkt["channel"]]})
        d["count"] += 1
        d["offsets"].append(pkt["cfo_khz"] * 1e3)
        if np.isfinite(pkt.get("snr_db", np.nan)):
            d["snrs"].append(pkt["snr_db"])
        d["types"].add(pkt["type"])
        d["last"] = snapshot
        for k, v in pkt.get("ad", []):
            if k in ("Name", "Short name"):
                d["name"] = v
            elif k == "Manufacturer data" and not d["maker"]:
                d["maker"] = v.split(":")[0] + (f" {v.split('(')[-1].rstrip(')')}" if "(" in v else "")
            elif k in ("Service data", "UUID16", "UUID16 (partial)") and not d["maker"]:
                label = v.split(":")[0].split(",")[0].split(" ", 1)
                if len(label) > 1:
                    d["maker"] = label[1]

    def receiver_offset(self):
        """Median of per-device mean offsets: a rough estimate of the ESP32's own error.
        Needs several devices; any one device's crystal can be off by up to 50 ppm."""
        means = [np.mean(d["offsets"]) for d in self.devices.values() if d["offsets"]]
        return float(np.median(means)) if len(means) >= 3 else None

    def table(self):
        if not self.devices:
            return "No devices yet."
        ref = self.receiver_offset()
        rows = [f"{'address':<17}  {'pkts':>4}  {'offset kHz':>10}  {'spread':>6}  "
                f"{'ppm':>6}  {'SNR dB':>6}  device"]
        for addr, d in sorted(self.devices.items(), key=lambda kv: -kv[1]["count"]):
            off = np.array(d["offsets"])
            rel = (off.mean() - (ref or 0)) / d["fchan"] * 1e6
            snr = f"{np.median(d['snrs']):.0f}" if d["snrs"] else "-"
            who = " / ".join(x for x in (d["name"], d["maker"]) if x) or ", ".join(sorted(d["types"]))
            rows.append(f"{addr:<17}  {d['count']:>4}  {off.mean() / 1e3:>+10.1f}  "
                        f"{off.std() / 1e3:>6.1f}  {rel:>+6.1f}  {snr:>6}  {who}")
        note = (f"\nppm is each device's crystal error after removing the median offset "
                f"({ref / 1e3:+.1f} kHz), which is mostly this ESP32's own error."
                if ref is not None else "\nppm is relative to 0 Hz (need 3+ devices to remove the ESP32's own error).")
        return "\n".join(rows) + note


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--fs", type=float, required=True, help="sample rate in MS/s (e.g. 80)")
    ap.add_argument("--fc", type=float, required=True, help="center frequency in MHz (e.g. 2441)")
    ap.add_argument("--format", choices=["cf32", "ci8", "cu8", "ci16", "npy", "wav"])
    ap.add_argument("--threshold", type=float, default=0.6, help="sync match threshold 0-1")
    ap.add_argument("--all", action="store_true", help="also show packets that fail CRC")
    ap.add_argument("--flip", choices=["auto", "no", "yes"], default="auto",
                    help="I/Q swapped? auto tries both")
    ap.add_argument("--summary", action="store_true", help="only print a per-device table")
    a = ap.parse_args()
    total = good = 0
    tracker = DeviceTracker()
    for i, path in enumerate(a.files):
        iq = load_iq(path, a.format)
        for pkt in decode_capture(iq, a.fs * 1e6, a.fc * 1e6, a.threshold, flip=a.flip):
            total += 1
            good += pkt["status"] == "CRC OK"
            tracker.add(pkt, i)
            if not a.summary and (a.all or pkt["status"] == "CRC OK"):
                print_packet(pkt, f"{os.path.basename(path)}: " if len(a.files) > 1 else "")
    print(f"\n{good} packets passed CRC out of {total} sync matches in {len(a.files)} file(s)\n")
    print(tracker.table())


if __name__ == "__main__":
    main()
