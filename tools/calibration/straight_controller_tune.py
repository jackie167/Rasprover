#!/usr/bin/env python3

import argparse
import math
import re
import statistics
import sys
import threading
import time
from pathlib import Path


SLAM_DEBUG_RE = re.compile(
    r"slam_debug dt=(?P<dt>-?\d+(?:\.\d+)?) "
    r"odl=(?P<odl>-?\d+(?:\.\d+)?) odr=(?P<odr>-?\d+(?:\.\d+)?) "
    r"dodl=(?P<dodl>-?\d+(?:\.\d+)?) dodr=(?P<dodr>-?\d+(?:\.\d+)?) "
    r"gyro=\[(?P<gx>-?\d+(?:\.\d+)?) (?P<gy>-?\d+(?:\.\d+)?) (?P<gz>-?\d+(?:\.\d+)?)\]"
)

STRAIGHT_CTRL_RE = re.compile(
    r"straight_ctrl heading_err=(?P<heading_err>-?\d+(?:\.\d+)?) "
    r"balance_err=(?P<balance_err>-?\d+(?:\.\d+)?) "
    r"gz=(?P<gz>-?\d+(?:\.\d+)?) "
    r"integral=(?P<integral>-?\d+(?:\.\d+)?) "
    r"correction=(?P<correction>-?\d+(?:\.\d+)?)"
)

MOTION_RE = re.compile(
    r"motion source=(?P<source>\S+) linear=(?P<linear>-?\d+(?:\.\d+)?) "
    r"(?:(?:angular_in=(?P<angular_in>-?\d+(?:\.\d+)?) angular_out=(?P<angular_out>-?\d+(?:\.\d+)?) "
    r"correction=(?P<correction>-?\d+(?:\.\d+)?) )|(?:angular=(?P<angular>-?\d+(?:\.\d+)?) ))"
    r"left=(?P<left>-?\d+(?:\.\d+)?) right=(?P<right>-?\d+(?:\.\d+)?)"
)


def pick_default_log_file():
    root = Path(__file__).resolve().parents[2] / "runtime_logs"
    candidates = [
        root / "ros_motion_base.log",
        root / "ros_slam_base.log",
        root / "ros_path_base.log",
    ]
    existing = [path for path in candidates if path.exists()]
    if not existing:
        return candidates[0]
    return max(existing, key=lambda path: path.stat().st_mtime)


def parse_float_map(match):
    parsed = {}
    for key, value in match.groupdict().items():
        if value is None:
            continue
        try:
            parsed[key] = float(value)
        except ValueError:
            parsed[key] = value
    return parsed


def collect_window(log_file, capture_seconds, timeout_seconds):
    start_offset = log_file.stat().st_size if log_file.exists() else 0
    print(f"[straight_tune] log file: {log_file}")
    print("[straight_tune] nhan Enter de bat dau thu log, chay robot, roi go s + Enter de ket thuc va tinh.")
    input("[straight_tune] press Enter to start capture: ")
    if capture_seconds > 0:
        print(
            f"[straight_tune] dang thu log. Go s + Enter de ket thuc khi robot da chay xong. "
            f"Tu dong timeout sau {capture_seconds:.1f}s."
        )
    else:
        print("[straight_tune] dang thu log. Go s + Enter de ket thuc khi robot da chay xong. Khong co auto-timeout.")

    stop_event = threading.Event()

    def wait_for_stop():
        while True:
            try:
                line = input().strip().lower()
            except EOFError:
                return
            if line == "s":
                stop_event.set()
                return

    threading.Thread(target=wait_for_stop, daemon=True).start()

    slam_rows = []
    straight_rows = []
    motion_rows = []
    deadline = time.time() + timeout_seconds
    capture_end = None if capture_seconds <= 0 else (time.time() + capture_seconds)
    current_offset = start_offset
    stop_reason = "timeout"

    while time.time() < deadline:
        if log_file.exists():
            with log_file.open("r", encoding="utf-8", errors="ignore") as handle:
                handle.seek(current_offset)
                chunk = handle.read()
                current_offset = handle.tell()
            for line in chunk.splitlines():
                match = SLAM_DEBUG_RE.search(line)
                if match:
                    slam_rows.append(parse_float_map(match))
                    continue
                match = STRAIGHT_CTRL_RE.search(line)
                if match:
                    straight_rows.append(parse_float_map(match))
                    continue
                match = MOTION_RE.search(line)
                if match:
                    motion_rows.append(parse_float_map(match))

        if stop_event.is_set():
            stop_reason = "user_enter"
            break
        if capture_end is not None and time.time() >= capture_end:
            stop_reason = "capture_timeout"
            break
        time.sleep(0.1)

    if stop_reason == "user_enter":
        time.sleep(0.5)
        if log_file.exists():
            with log_file.open("r", encoding="utf-8", errors="ignore") as handle:
                handle.seek(current_offset)
                chunk = handle.read()
                current_offset = handle.tell()
            for line in chunk.splitlines():
                match = SLAM_DEBUG_RE.search(line)
                if match:
                    slam_rows.append(parse_float_map(match))
                    continue
                match = STRAIGHT_CTRL_RE.search(line)
                if match:
                    straight_rows.append(parse_float_map(match))
                    continue
                match = MOTION_RE.search(line)
                if match:
                    motion_rows.append(parse_float_map(match))

    return slam_rows, straight_rows, motion_rows, stop_reason


def mean_or_none(values):
    return statistics.fmean(values) if values else None


def stdev_or_none(values):
    return statistics.pstdev(values) if len(values) >= 2 else None


def infer_drift(avg_balance_err):
    if avg_balance_err is None:
        return "unknown"
    if avg_balance_err > 0.01:
        return "left"
    if avg_balance_err < -0.01:
        return "right"
    return "straightish"


def format_float(value, digits=4):
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def total_run_metrics(slam_rows):
    if len(slam_rows) < 2:
        return {}

    start = slam_rows[0]
    end = slam_rows[-1]
    start_odl = start["odl"] - start["dodl"]
    start_odr = start["odr"] - start["dodr"]
    end_odl = end["odl"]
    end_odr = end["odr"]
    delta_left = end_odl - start_odl
    delta_right = end_odr - start_odr
    delta_distance = 0.5 * (delta_left + delta_right)
    delta_yaw_deg = math.degrees((delta_right - delta_left) / 0.52) if abs(0.52) > 1e-9 else None
    left_right_ratio_total = delta_left / delta_right if abs(delta_right) > 1e-9 else None

    return {
        "start_odl": start_odl,
        "start_odr": start_odr,
        "end_odl": end_odl,
        "end_odr": end_odr,
        "delta_left": delta_left,
        "delta_right": delta_right,
        "delta_distance": delta_distance,
        "delta_yaw_deg": delta_yaw_deg,
        "left_right_ratio_total": left_right_ratio_total,
    }


def recommend_adjustments(avg_balance_err, avg_correction, max_abs_correction, args):
    recommendations = []
    drift = infer_drift(avg_balance_err)

    if drift == "left":
        recommendations.append("robot van sang trai")
        if max_abs_correction is not None and max_abs_correction >= args.max_correction_near_limit:
            recommendations.append(
                "controller da gan cham tran correction: tang STRAIGHT_CONTROLLER_MAX_CORRECTION truoc"
            )
        else:
            recommendations.append(
                "tang STRAIGHT_CONTROLLER_HEADING_GAIN hoac STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN"
            )
        if avg_correction is not None and avg_correction <= 0.0:
            recommendations.append("dau correction co ve chua dung huong: giam gyro gain va kiem tra wheel_yaw_scale")
    elif drift == "right":
        recommendations.append("robot van sang phai")
        if max_abs_correction is not None and max_abs_correction >= args.max_correction_near_limit:
            recommendations.append(
                "controller da gan cham tran correction: tang STRAIGHT_CONTROLLER_MAX_CORRECTION truoc"
            )
        else:
            recommendations.append(
                "tang STRAIGHT_CONTROLLER_HEADING_GAIN hoac STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN"
            )
        if avg_correction is not None and avg_correction >= 0.0:
            recommendations.append("dau correction co ve chua dung huong: giam gyro gain va kiem tra wheel_yaw_scale")
    else:
        recommendations.append("khong thay xu huong lech ro rang trong doan test nay")
        if avg_correction is not None and abs(avg_correction) > 0.03:
            recommendations.append("controller dang bua nhieu du robot kha thang: co the giam nhe heading_gain")

    return recommendations


def main():
    parser = argparse.ArgumentParser(
        description="Capture a straight-line controller test from robot_base_node logs and suggest next gain changes."
    )
    parser.add_argument(
        "--log-file",
        default=str(pick_default_log_file()),
        help="Base-node runtime log to watch. Default picks the newest ros_*_base.log.",
    )
    parser.add_argument(
        "--capture-seconds",
        type=float,
        default=12.0,
        help="How long to capture after arming. Keep this close to your 2m straight run duration.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=20.0,
        help="Hard timeout while waiting for appended log lines.",
    )
    parser.add_argument(
        "--max-correction-near-limit",
        type=float,
        default=0.10,
        help="Treat the controller as near saturation if max abs correction reaches this value.",
    )
    args = parser.parse_args()

    log_file = Path(args.log_file).expanduser().resolve()
    if not log_file.exists():
        raise SystemExit(f"log file not found: {log_file}")

    slam_rows, straight_rows, motion_rows, stop_reason = collect_window(
        log_file=log_file,
        capture_seconds=args.capture_seconds,
        timeout_seconds=args.timeout_seconds,
    )

    if not slam_rows and not straight_rows and not motion_rows:
        raise SystemExit("[straight_tune] khong thu duoc dong log nao. Hay kiem tra stack dang chay va log base dang cap nhat.")

    avg_dodl = mean_or_none([row["dodl"] for row in slam_rows])
    avg_dodr = mean_or_none([row["dodr"] for row in slam_rows])
    avg_gz = mean_or_none([row["gz"] for row in slam_rows])
    avg_balance_err = mean_or_none([row["balance_err"] for row in straight_rows])
    avg_heading_err = mean_or_none([row["heading_err"] for row in straight_rows])
    avg_correction = mean_or_none([row["correction"] for row in straight_rows])
    correction_std = stdev_or_none([row["correction"] for row in straight_rows])
    max_abs_correction = max((abs(row["correction"]) for row in straight_rows), default=None)
    avg_linear = mean_or_none([row["linear"] for row in motion_rows if abs(row.get("linear", 0.0)) > 1e-6])
    totals = total_run_metrics(slam_rows)

    left_right_ratio = None
    if avg_dodr is not None and abs(avg_dodr) > 1e-6 and avg_dodl is not None:
        left_right_ratio = avg_dodl / avg_dodr

    drift = infer_drift(avg_balance_err)
    recommendations = recommend_adjustments(avg_balance_err, avg_correction, max_abs_correction, args)

    print("")
    print("[straight_tune] tom tat")
    print(f"  stop_reason: {stop_reason}")
    print(f"  samples slam_debug: {len(slam_rows)}")
    print(f"  samples straight_ctrl: {len(straight_rows)}")
    print(f"  samples motion: {len(motion_rows)}")
    print(f"  avg_linear: {format_float(avg_linear, 3)}")
    print(f"  start_odl: {format_float(totals.get('start_odl'), 6)}")
    print(f"  start_odr: {format_float(totals.get('start_odr'), 6)}")
    print(f"  end_odl: {format_float(totals.get('end_odl'), 6)}")
    print(f"  end_odr: {format_float(totals.get('end_odr'), 6)}")
    print(f"  delta_left_total: {format_float(totals.get('delta_left'), 6)}")
    print(f"  delta_right_total: {format_float(totals.get('delta_right'), 6)}")
    print(f"  delta_distance_total: {format_float(totals.get('delta_distance'), 6)}")
    print(f"  delta_yaw_deg_total: {format_float(totals.get('delta_yaw_deg'), 3)}")
    print(f"  left_right_ratio_total: {format_float(totals.get('left_right_ratio_total'), 4)}")
    print(f"  avg_dodl: {format_float(avg_dodl, 6)}")
    print(f"  avg_dodr: {format_float(avg_dodr, 6)}")
    print(f"  left_right_ratio: {format_float(left_right_ratio, 4)}")
    print(f"  avg_gz: {format_float(avg_gz, 4)}")
    print(f"  avg_heading_err: {format_float(avg_heading_err, 4)}")
    print(f"  avg_balance_err: {format_float(avg_balance_err, 4)}")
    print(f"  avg_correction: {format_float(avg_correction, 4)}")
    print(f"  correction_std: {format_float(correction_std, 4)}")
    print(f"  max_abs_correction: {format_float(max_abs_correction, 4)}")
    print(f"  inferred_drift: {drift}")
    if motion_rows:
        first_motion = motion_rows[0]
        last_motion = motion_rows[-1]
        print(
            "  first_motion: linear=%s left=%s right=%s"
            % (
                format_float(first_motion.get("linear"), 3),
                format_float(first_motion.get("left"), 3),
                format_float(first_motion.get("right"), 3),
            )
        )
        print(
            "  last_motion: linear=%s left=%s right=%s"
            % (
                format_float(last_motion.get("linear"), 3),
                format_float(last_motion.get("left"), 3),
                format_float(last_motion.get("right"), 3),
            )
        )

    print("")
    print("[straight_tune] nhan dinh")
    for item in recommendations:
        print(f"  - {item}")

    print("")
    print("[straight_tune] goi y tune")
    if drift == "left":
        print("  - thu tang STRAIGHT_CONTROLLER_HEADING_GAIN tu 0.90 len 1.05")
        print("  - neu correction da lon san, tang STRAIGHT_CONTROLLER_MAX_CORRECTION tu 0.12 len 0.15")
        print("  - neu encoder lech on dinh, tang STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN tu 0.80 len 1.00")
    elif drift == "right":
        print("  - thu tang STRAIGHT_CONTROLLER_HEADING_GAIN tu 0.90 len 1.05")
        print("  - neu correction da lon san, tang STRAIGHT_CONTROLLER_MAX_CORRECTION tu 0.12 len 0.15")
        print("  - neu encoder lech on dinh, tang STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN tu 0.80 len 1.00")
    else:
        print("  - neu duong chay thuc te da on, giu gain hien tai")
        print("  - neu robot van lac trai-phai, giam STRAIGHT_CONTROLLER_HEADING_GAIN xuong 0.75")

    print("")
    print("[straight_tune] vi du")
    print("  export STRAIGHT_CONTROLLER_HEADING_GAIN=1.05")
    print("  export STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN=1.00")
    print("  export STRAIGHT_CONTROLLER_MAX_CORRECTION=0.15")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[straight_tune] stopped by user")
        sys.exit(130)
