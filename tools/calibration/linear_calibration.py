#!/usr/bin/env python3

import argparse
import csv
import glob
import math
import sys
import time
from pathlib import Path


def ensure_ros_package_path():
    repo_root = Path(__file__).resolve().parents[2]
    package_root = repo_root / "ros2_ws" / "src" / "rasprover_base"
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))


ensure_ros_package_path()

from rasprover_base.base_driver import BaseDriver


DEFAULT_BOOT_COMMANDS = (
    {"T": 142, "cmd": 50},
    {"T": 131, "cmd": 1},
    {"T": 143, "cmd": 0},
)


def detect_port():
    for path in ("/dev/serial0", "/dev/ttyAMA0"):
        if glob.glob(path):
            return path
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/serial*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def select_with_timeout(timeout_s):
    import select

    return bool(select.select([sys.stdin], [], [], timeout_s)[0])


def packet_float(packet, key):
    try:
        return float(packet[key])
    except (KeyError, TypeError, ValueError):
        return None


def read_feedback_packet(driver, timeout=2.0):
    deadline = time.time() + timeout
    latest = None
    while time.time() < deadline:
        packet = driver.get_feedback()
        if isinstance(packet, dict) and packet.get("T") == 1001:
            latest = packet
            if "odl" in packet and "odr" in packet:
                return latest
        time.sleep(0.01)
    return latest


def compute_stage_metrics(initial_packet, current_packet, wheel_separation_m):
    start_left = packet_float(initial_packet, "odl")
    start_right = packet_float(initial_packet, "odr")
    current_left = packet_float(current_packet, "odl")
    current_right = packet_float(current_packet, "odr")
    if None in (start_left, start_right, current_left, current_right):
        return None

    d_left = current_left - start_left
    d_right = current_right - start_right
    estimated_distance = (d_left + d_right) * 0.5
    estimated_yaw_deg = math.degrees((d_right - d_left) / max(wheel_separation_m, 1e-6))
    left_right_ratio = d_left / d_right if abs(d_right) > 1e-6 else None

    return {
        "delta_left": d_left,
        "delta_right": d_right,
        "estimated_distance": estimated_distance,
        "estimated_yaw_deg": estimated_yaw_deg,
        "left_right_ratio": left_right_ratio,
    }


def prompt_yes(target_m):
    while True:
        answer = input(f"[linear_calib] ready for target {target_m:.3f} m? type y then Enter: ").strip().lower()
        if answer == "y":
            return
        print("[linear_calib] please type y to start this stage.")


def prompt_actual_distance(default_target_m):
    raw = input(
        f"[linear_calib] measured actual distance in meters (Enter to use target {default_target_m:.3f}): "
    ).strip()
    if not raw:
        return default_target_m
    return float(raw)


def run_translation_manual(driver, left_speed, right_speed, wheel_separation_m, timeout_s):
    initial_packet = read_feedback_packet(driver, timeout=3.0)
    if initial_packet is None:
        raise RuntimeError("No T=1001 feedback with odl/odr before starting translation.")

    driver.send_lr(left_speed, right_speed)
    start_time = time.time()
    latest_packet = initial_packet
    last_command_time = start_time

    print("[linear_calib] driving now. Press Enter when the robot reaches the physical target distance.")
    try:
        while time.time() - start_time < timeout_s:
            now = time.time()
            if now - last_command_time >= 0.1:
                driver.send_lr(left_speed, right_speed)
                last_command_time = now
            packet = read_feedback_packet(driver, timeout=0.05)
            if packet is not None:
                latest_packet = packet
            if select_with_timeout(0.0):
                sys.stdin.readline()
                break
    finally:
        driver.stop()
        time.sleep(0.25)
        driver.stop()

    metrics = compute_stage_metrics(initial_packet, latest_packet, wheel_separation_m)
    return {
        "duration_s": time.time() - start_time,
        "last_packet": latest_packet,
        "metrics": metrics,
    }


def main():
    parser = argparse.ArgumentParser(description="Interactive straight-line calibration using wheel odometry.")
    parser.add_argument("--port", default="", help="Serial port to open. Auto-detect if omitted.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument(
        "--speed",
        type=float,
        default=0.12,
        help="Wheel speed magnitude used for straight motion. Use a low value for stable calibration.",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Drive backward instead of forward during the calibration stages.",
    )
    parser.add_argument(
        "--wheel-separation-m",
        type=float,
        default=0.52,
        help="Effective wheel separation used only to estimate heading drift during the stage.",
    )
    parser.add_argument("--timeout", type=float, default=25.0, help="Per-stage timeout in seconds.")
    parser.add_argument(
        "--targets",
        default="1.0,2.0",
        help="Comma-separated target distances in meters. These are only prompts; stop is manual.",
    )
    parser.add_argument("--skip-init", action="store_true", help="Do not send startup commands.")
    parser.add_argument(
        "--output",
        default="",
        help="Optional CSV output path. Default is a timestamped file in tools/calibration/captures/.",
    )
    args = parser.parse_args()

    port = args.port or detect_port()
    if not port:
        raise SystemExit("No serial device found. Tried /dev/serial0, /dev/ttyAMA0, /dev/ttyUSB*, /dev/ttyACM*.")

    targets = []
    for item in args.targets.split(","):
        item = item.strip()
        if not item:
            continue
        targets.append(float(item))
    if not targets:
        raise SystemExit("No valid targets given.")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    default_dir = Path(__file__).resolve().parent / "captures"
    default_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output or str(default_dir / f"linear_calibration_{timestamp}.csv")

    signed_speed = -abs(args.speed) if args.reverse else abs(args.speed)
    print(
        f"[linear_calib] port={port} baud={args.baud} targets={targets} "
        f"speed={signed_speed:.3f} wheel_separation_m={args.wheel_separation_m:.3f}"
    )
    print("[linear_calib] mode=manual-stop; robot drives straight until you press Enter at the physical target distance")
    print("[linear_calib] stop ROS stacks first so the serial port is free.")

    driver = BaseDriver(port, args.baud)
    rows = []

    try:
        if not args.skip_init:
            for cmd in DEFAULT_BOOT_COMMANDS:
                print(f"[linear_calib] send {cmd}")
                driver.send_json(cmd)
                time.sleep(0.15)

        for target_m in targets:
            prompt_yes(target_m)
            print(f"[linear_calib] driving toward physical target {target_m:.3f} m")
            result = run_translation_manual(
                driver,
                left_speed=signed_speed,
                right_speed=signed_speed,
                wheel_separation_m=args.wheel_separation_m,
                timeout_s=args.timeout,
            )
            metrics = result["metrics"]
            if metrics is None:
                raise RuntimeError("Could not compute stage metrics from odl/odr.")

            actual_distance_m = prompt_actual_distance(target_m)
            estimated_distance_m = metrics["estimated_distance"]
            scale_stage = None
            if abs(estimated_distance_m) > 1e-6:
                scale_stage = actual_distance_m / estimated_distance_m

            row = {
                "target_distance_m": target_m,
                "estimated_distance_m": estimated_distance_m,
                "actual_measured_distance_m": actual_distance_m,
                "recommended_linear_scale_stage": scale_stage,
                "delta_odl": metrics["delta_left"],
                "delta_odr": metrics["delta_right"],
                "estimated_yaw_deg": metrics["estimated_yaw_deg"],
                "left_right_ratio": metrics["left_right_ratio"],
                "speed": signed_speed,
                "wheel_separation_m": args.wheel_separation_m,
                "duration_s": result["duration_s"],
            }
            rows.append(row)

            print(
                "[linear_calib] stage result: "
                f"estimated_distance_m={estimated_distance_m:.6f} "
                f"actual_distance_m={actual_distance_m:.6f} "
                f"recommended_linear_scale_stage={scale_stage:.6f} "
                f"estimated_yaw_deg={metrics['estimated_yaw_deg']:.3f} "
                f"left_right_ratio={metrics['left_right_ratio']!s}"
            )

        valid_scales = [row["recommended_linear_scale_stage"] for row in rows if row["recommended_linear_scale_stage"]]
        overall_scale = sum(valid_scales) / len(valid_scales) if valid_scales else None

        with open(output_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "target_distance_m",
                    "estimated_distance_m",
                    "actual_measured_distance_m",
                    "recommended_linear_scale_stage",
                    "delta_odl",
                    "delta_odr",
                    "estimated_yaw_deg",
                    "left_right_ratio",
                    "speed",
                    "wheel_separation_m",
                    "duration_s",
                ],
            )
            writer.writeheader()
            writer.writerows(rows)

        print(f"[linear_calib] wrote CSV: {output_path}")
        if overall_scale is not None:
            print(f"[linear_calib] overall recommended LINEAR_ODOM_SCALE={overall_scale:.6f}")
        else:
            print("[linear_calib] no valid overall scale computed.")
    finally:
        try:
            driver.stop()
        except Exception:
            pass
        try:
            driver.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
