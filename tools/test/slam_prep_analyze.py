#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
from collections import defaultdict


def parse_float(value):
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_rows(path):
    rows = []
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            parsed = dict(row)
            for key in (
                "wall_time",
                "elapsed",
                "command_left",
                "command_right",
                "packet_type",
                "L",
                "R",
                "odl",
                "odr",
                "gx",
                "gy",
                "gz",
                "ax",
                "ay",
                "az",
                "mx",
                "my",
                "mz",
                "v",
            ):
                parsed[key] = parse_float(parsed.get(key))
            rows.append(parsed)
    return rows


def stage_rows(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["stage"]].append(row)
    return grouped


def series(rows, key):
    return [row[key] for row in rows if row.get(key) is not None]


def summarize_mean_std(values):
    if not values:
        return None, None
    mean = statistics.fmean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return mean, std


def delta_from_stage(rows, key):
    values = series(rows, key)
    if len(values) < 2:
        return None
    return values[-1] - values[0]


def print_stationary_summary(rows):
    print("Stationary summary")
    for key in ("gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz"):
        mean, std = summarize_mean_std(series(rows, key))
        if mean is None:
            continue
        print("  %s mean=%.6f std=%.6f" % (key, mean, std))


def print_motion_summary(stage_name, rows):
    elapsed = series(rows, "elapsed")
    duration = elapsed[-1] - elapsed[0] if len(elapsed) >= 2 else 0.0
    dodl = delta_from_stage(rows, "odl")
    dodr = delta_from_stage(rows, "odr")
    gz_mean, gz_std = summarize_mean_std(series(rows, "gz"))
    ax_mean, ax_std = summarize_mean_std(series(rows, "ax"))
    ay_mean, ay_std = summarize_mean_std(series(rows, "ay"))
    print("Motion summary: %s" % stage_name)
    print("  duration=%.3f s" % duration)
    if dodl is not None and dodr is not None:
        print("  delta_odl=%.6f delta_odr=%.6f diff=%.6f ratio=%.6f" % (
            dodl,
            dodr,
            dodl - dodr,
            (dodl / dodr) if dodr not in (None, 0.0) else math.nan,
        ))
    if gz_mean is not None:
        print("  gz mean=%.6f std=%.6f" % (gz_mean, gz_std))
    if ax_mean is not None:
        print("  ax mean=%.6f std=%.6f" % (ax_mean, ax_std))
    if ay_mean is not None:
        print("  ay mean=%.6f std=%.6f" % (ay_mean, ay_std))


def detect_profile_stages(grouped):
    return {
        "idle": grouped.get("idle", []),
        "forward": grouped.get("forward", []),
        "reverse": grouped.get("reverse", []),
        "spin_left": grouped.get("spin_left", []),
        "spin_right": grouped.get("spin_right", []),
    }


def main():
    parser = argparse.ArgumentParser(description="Analyze captured IMU/encoder data for SLAM preparation.")
    parser.add_argument("--input", required=True, help="CSV captured by slam_prep_capture.py")
    args = parser.parse_args()

    rows = load_rows(args.input)
    if not rows:
        raise SystemExit("No rows found in %s" % args.input)

    grouped = stage_rows(rows)
    stages = detect_profile_stages(grouped)

    print("File: %s" % args.input)
    print("Rows: %d" % len(rows))
    print("Packet types: %s" % sorted({int(row["packet_type"]) for row in rows if row.get("packet_type") is not None}))

    if stages["idle"]:
        print_stationary_summary(stages["idle"])
    else:
        print("Stationary summary")
        print("  idle stage not found")

    for stage_name in ("forward", "reverse", "spin_left", "spin_right"):
        if stages[stage_name]:
            print_motion_summary(stage_name, stages[stage_name])

    print("Interpretation hints")
    print("  If forward delta_odl and delta_odr are close, encoder scaling left/right is roughly symmetric.")
    print("  If spin_left and spin_right produce opposite-signed deltas, encoder sign is consistent for in-place yaw.")
    print("  If stationary gyro std is small and mean is near zero, gyro bias/noise is good enough for EKF start.")
    print("  If accel means are stable while stationary, that helps infer axis orientation and unit consistency.")


if __name__ == "__main__":
    main()
