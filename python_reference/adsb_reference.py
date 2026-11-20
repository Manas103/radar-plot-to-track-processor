#!/usr/bin/env python3
"""Independent Python re-implementation of the Mode S CRC-24 and local
CPR decode algorithms adsb.cpp implements, written from scratch with no
shared code. This cross-checks the C++ side's decode output against a
second, independent implementation of the same public algorithm; it does
not validate against externally captured real ADS-B traffic, which this
machine has no receiver for (see the README "Honest framing").
"""
from __future__ import annotations

import math
import sys

GENERATOR_POLYNOMIAL = 0xFFF409


def crc24(frame: bytes) -> int:
    reg = 0
    bits = []
    for byte in frame[:11]:  # first 88 bits: DF+CA+ICAO+ME
        for i in range(7, -1, -1):
            bits.append((byte >> i) & 1)
    for bit in bits:
        top_bit = (reg >> 23) & 1
        reg = ((reg << 1) | bit) & 0xFFFFFF
        if top_bit:
            reg ^= GENERATOR_POLYNOMIAL
    return reg & 0xFFFFFF


def get_bits(frame: bytes, start_bit: int, num_bits: int) -> int:
    value = 0
    for i in range(num_bits):
        global_bit = start_bit + i
        byte_idx = global_bit // 8
        bit_idx = 7 - (global_bit % 8)
        bit = (frame[byte_idx] >> bit_idx) & 1
        value = (value << 1) | bit
    return value


def cpr_nl(lat_deg: float) -> float:
    lat = abs(lat_deg)
    if lat >= 87.0:
        return 1.0
    if lat < 1e-9:
        return 59.0
    nz = 15.0
    a = 1.0 - math.cos(math.pi / (2.0 * nz))
    b = math.cos(math.pi / 180.0 * lat) ** 2
    val = 1.0 - a / b
    val = max(-1.0, min(1.0, val))
    return max(1.0, math.floor(2.0 * math.pi / math.acos(val)))


def wrap_mod(a: float, b: float) -> float:
    r = math.fmod(a, b)
    if r < 0.0:
        r += b
    return r


def decode_airborne_position(frame: bytes, ref_lat_deg: float, ref_lon_deg: float):
    computed = crc24(frame)
    transmitted = get_bits(frame, 88, 24)
    crc_ok = computed == transmitted
    icao24 = get_bits(frame, 8, 24)
    odd = get_bits(frame, 53, 1) != 0

    lat_cpr = get_bits(frame, 54, 17) / 131072.0
    lon_cpr = get_bits(frame, 71, 17) / 131072.0

    dlat = 360.0 / 59.0 if odd else 360.0 / 60.0
    ref_lat_mod = wrap_mod(ref_lat_deg, dlat)
    j = math.floor(ref_lat_deg / dlat) + round(ref_lat_mod / dlat - lat_cpr)
    lat = dlat * (j + lat_cpr)

    nl = cpr_nl(lat)
    ni = max(nl - (1.0 if odd else 0.0), 1.0)
    dlon = 360.0 / ni
    ref_lon_mod = wrap_mod(ref_lon_deg, dlon)
    m = math.floor(ref_lon_deg / dlon) + round(ref_lon_mod / dlon - lon_cpr)
    lon = dlon * (m + lon_cpr)

    return {"crc_ok": crc_ok, "icao24": icao24, "odd": odd, "lat_deg": lat, "lon_deg": lon}


def run_cross_check(frames_path: str) -> dict:
    total = 0
    crc_agree = 0
    position_agree = 0
    max_err_deg = 0.0
    with open(frames_path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            icao_hex, true_lat, true_lon, odd_flag, frame_hex, ref_lat, ref_lon = parts
            frame = bytes.fromhex(frame_hex)
            result = decode_airborne_position(frame, float(ref_lat), float(ref_lon))
            total += 1
            if result["crc_ok"]:
                crc_agree += 1
            err = math.hypot(result["lat_deg"] - float(true_lat), result["lon_deg"] - float(true_lon))
            max_err_deg = max(max_err_deg, err)
            # ~5m of CPR quantization is on the order of 4.5e-5 degrees; a
            # correct independent decode should land well inside 1e-3 deg.
            if err < 1e-3:
                position_agree += 1
    return {
        "total": total,
        "crc_agree": crc_agree,
        "position_agree": position_agree,
        "max_err_deg": max_err_deg,
    }


if __name__ == "__main__":
    result = run_cross_check(sys.argv[1])
    print(f"total={result['total']}")
    print(f"crc_agree={result['crc_agree']}/{result['total']}")
    print(f"position_agree={result['position_agree']}/{result['total']}")
    print(f"max_err_deg={result['max_err_deg']:.9f}")
    if result["crc_agree"] != result["total"] or result["position_agree"] != result["total"]:
        sys.exit(1)
