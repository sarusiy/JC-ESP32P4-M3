#!/usr/bin/env python3
"""Triage the CAN IDs in a capture: classify each by timing (periodic vs
event-like) and per-byte liveness (static / counter-like / live), then rank
the ones NOT already in signal_dictionary.json so a human knows what's
actually worth looking at next instead of scrolling raw hex.

Usage:
    python can_id_triage.py CAPTURE.csv [CAPTURE2.log ...] [--dict PATH] [--top N]

Accepts our app's CSV format (phone_time_ms,board_time_us,...,can_id,...,data_hex,...)
and the candump/socketCAN .log format ("(seconds.micros) can0 id#data").

This script only READS signal_dictionary.json (to filter out already-known
IDs) -- it never writes to it. Promote a finding into the dictionary by hand
once it's actually confirmed, so the dictionary stays a record of verified
knowledge, not machine guesses.
"""
import argparse
import csv
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def parse_csv(path):
    frames = []
    with open(path, encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            can_id = int(row["can_id"], 16)
            ts = int(row["board_time_us"]) / 1_000_000.0
            data = bytes.fromhex(row["data_hex"].strip())
            frames.append((ts, can_id, data))
    return frames


def parse_candump(path):
    frames = []
    with open(path, encoding="ascii") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("("):
                continue
            ts_str, rest = line[1:].split(")", 1)
            ts = float(ts_str)
            _iface, frame = rest.strip().split(" ", 1)
            id_str, data_str = frame.split("#", 1)
            can_id = int(id_str, 16)
            data = bytes.fromhex(data_str.strip())
            frames.append((ts, can_id, data))
    return frames


def load_frames(paths):
    frames = []
    for path in paths:
        path = Path(path)
        if path.suffix.lower() == ".csv":
            frames.extend(parse_csv(path))
        else:
            frames.extend(parse_candump(path))
    frames.sort(key=lambda f: f[0])
    return frames


def load_dictionary(dict_path):
    if not dict_path or not Path(dict_path).exists():
        return {}
    with open(dict_path, encoding="utf-8") as f:
        data = json.load(f)
    return {int(k, 16): v for k, v in data.get("ids", {}).items()}


def classify_timing(timestamps):
    count = len(timestamps)
    if count < 2:
        return "single", 0.0, 0.0
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    mean_interval = statistics.mean(intervals)
    stdev_interval = statistics.pstdev(intervals) if len(intervals) > 1 else 0.0
    if count < 5:
        label = "rare"
    elif mean_interval > 0 and stdev_interval / mean_interval < 0.25:
        label = "periodic"
    else:
        label = "bursty/irregular"
    return label, mean_interval, stdev_interval


def analyze_bytes(payloads):
    """Returns per-byte-position info: static / counter-like / live, plus a
    live-byte count used for ranking. Ragged DLC across frames of the same
    ID is handled by only analyzing positions present in every frame."""
    if not payloads:
        return [], 0
    min_len = min(len(p) for p in payloads)
    result = []
    live_count = 0
    for pos in range(min_len):
        values = [p[pos] for p in payloads]
        distinct = set(values)
        if len(distinct) == 1:
            result.append((pos, "static", values[0]))
            continue
        # counter-like: consecutive frame-to-frame delta is +1 (mod 256) most of the time
        deltas = [(b - a) % 256 for a, b in zip(values, values[1:])]
        if deltas and sum(1 for d in deltas if d == 1) / len(deltas) > 0.8:
            result.append((pos, "counter-like", len(distinct)))
            continue
        result.append((pos, "live", len(distinct)))
        live_count += 1
    return result, live_count


def format_id(can_id):
    return f"0x{can_id:X}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("captures", nargs="+", help="Capture file(s): .csv (our app format) or .log (candump format)")
    parser.add_argument("--dict", default=None, help="Path to signal_dictionary.json (default: none, nothing filtered)")
    parser.add_argument("--top", type=int, default=20, help="How many unknown IDs to print in the ranked report (default 20)")
    parser.add_argument("--check-ids", default=None,
                         help="Comma-separated hex IDs to specifically check for presence as senders (the 'is this module actually silent' check), independent of the ranked report")
    args = parser.parse_args()

    frames = load_frames(args.captures)
    if not frames:
        print("No frames parsed.", file=sys.stderr)
        sys.exit(1)

    known = load_dictionary(args.dict)

    by_id = defaultdict(list)
    for ts, can_id, data in frames:
        by_id[can_id].append((ts, data))

    total_span = frames[-1][0] - frames[0][0]
    print(f"Loaded {len(frames)} frames, {len(by_id)} distinct IDs, spanning {total_span:.1f}s\n")

    if args.check_ids:
        check_list = [int(x.strip(), 16) for x in args.check_ids.split(",") if x.strip()]
        print("=== Fixed-ID presence check ===")
        for cid in check_list:
            count = len(by_id.get(cid, []))
            status = f"PRESENT ({count} frames)" if count else "not seen"
            print(f"  {format_id(cid):>12}  {status}")
        print()

    rows = []
    for can_id, entries in by_id.items():
        timestamps = [e[0] for e in entries]
        payloads = [e[1] for e in entries]
        timing_label, mean_interval, stdev_interval = classify_timing(timestamps)
        byte_info, live_count = analyze_bytes(payloads)
        rows.append({
            "id": can_id,
            "count": len(entries),
            "timing": timing_label,
            "mean_interval_ms": mean_interval * 1000,
            "live_bytes": live_count,
            "byte_info": byte_info,
            "known": can_id in known,
        })

    unknown_rows = [r for r in rows if not r["known"]]
    known_rows = [r for r in rows if r["known"]]
    unknown_rows.sort(key=lambda r: (-r["live_bytes"], r["count"]))

    print(f"=== {len(known_rows)} already-known ID(s), filtered out of ranking (in --dict) ===")
    for r in sorted(known_rows, key=lambda r: r["id"]):
        print(f"  {format_id(r['id']):>12}  {known[r['id']].get('label', '')}")
    print()

    print(f"=== Top {min(args.top, len(unknown_rows))} unranked/unknown ID(s), ranked by live-byte count ===")
    print(f"{'ID':>12}  {'frames':>7}  {'timing':<16}  {'period(ms)':>11}  {'live bytes':>10}  detail")
    for r in unknown_rows[: args.top]:
        detail = ", ".join(
            f"b{pos}={kind}" + (f"({extra})" if kind != "static" else f"=0x{extra:02X}")
            for pos, kind, extra in r["byte_info"] if kind != "static"
        ) or "(all bytes static)"
        period = f"{r['mean_interval_ms']:.1f}" if r["timing"] != "single" else "-"
        print(f"{format_id(r['id']):>12}  {r['count']:>7}  {r['timing']:<16}  {period:>11}  {r['live_bytes']:>10}  {detail}")


if __name__ == "__main__":
    main()
