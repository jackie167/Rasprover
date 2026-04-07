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
from rasprover_base.state_store import StateStore


DEFAULT_BOOT_COMMANDS = (
    {"T": 142, "cmd": 50},
    {"T": 131, "cmd": 1},
    {"T": 143, "cmd": 0},
)


def detect_port():
    for path in ("/dev/ttyAMA0", "/dev/serial0"):
        if glob.glob(path):
            return path
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/serial*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def read_feedback_packet(driver, timeout=2.0):
    deadline = time.time() + timeout
    latest = None
    while time.time() < deadline:
        packet = driver.get_feedback()
        if isinstance(packet, dict):
            latest = packet
            packet_type = packet.get("T")
            try:
                packet_type = int(packet_type)
            except (TypeError, ValueError):
                packet_type = None
            if packet_type == 1001 and "odl" in packet and "odr" in packet:
                return latest
        time.sleep(0.01)
    return latest


def wait_for_encoder_feedback(driver, timeout=6.0):
    latest = read_feedback_packet(driver, timeout=timeout)
    if isinstance(latest, dict):
        packet_type = latest.get("T")
        try:
            packet_type = int(packet_type)
        except (TypeError, ValueError):
            packet_type = None
        if packet_type == 1001 and "odl" in latest and "odr" in latest:
            return latest
    return None


def packet_float(packet, key):
    try:
        return float(packet[key])
    except (KeyError, TypeError, ValueError):
        return None


def compute_delta_yaw_deg(initial_packet, current_packet, wheel_separation_m, yaw_scale):
    start_left = packet_float(initial_packet, "odl")
    start_right = packet_float(initial_packet, "odr")
    current_left = packet_float(current_packet, "odl")
    current_right = packet_float(current_packet, "odr")
    if None in (start_left, start_right, current_left, current_right):
        return None
    d_left = current_left - start_left
    d_right = current_right - start_right
    d_theta = (d_right - d_left) / max(wheel_separation_m, 1e-6)
    d_theta *= yaw_scale
    return math.degrees(d_theta)


def run_rotation_auto(driver, target_deg, spin_speed, wheel_separation_m, yaw_scale, timeout_s):
    initial_packet = wait_for_encoder_feedback(driver, timeout=6.0)
    if initial_packet is None:
        raise RuntimeError("No T=1001 feedback with odl/odr before starting rotation.")

    driver.send_lr(-spin_speed, spin_speed)
    start_time = time.time()
    latest_packet = initial_packet
    estimated_deg = 0.0
    last_command_time = start_time

    try:
        while time.time() - start_time < timeout_s:
            now = time.time()
            if now - last_command_time >= 0.1:
                driver.send_lr(-spin_speed, spin_speed)
                last_command_time = now
            packet = read_feedback_packet(driver, timeout=0.2)
            if packet is None:
                continue
            latest_packet = packet
            current_deg = compute_delta_yaw_deg(initial_packet, latest_packet, wheel_separation_m, yaw_scale)
            if current_deg is None:
                continue
            estimated_deg = current_deg
            if abs(estimated_deg) >= target_deg:
                break
    finally:
        driver.stop()
        time.sleep(0.25)
        driver.stop()

    return {
        "estimated_deg": estimated_deg,
        "duration_s": time.time() - start_time,
        "last_packet": latest_packet,
    }


def run_rotation_manual(driver, spin_speed, wheel_separation_m, yaw_scale, timeout_s):
    initial_packet = wait_for_encoder_feedback(driver, timeout=6.0)
    if initial_packet is None:
        raise RuntimeError("No T=1001 feedback with odl/odr before starting rotation.")

    driver.send_lr(-spin_speed, spin_speed)
    start_time = time.time()
    latest_packet = initial_packet
    estimated_deg = 0.0
    last_command_time = start_time

    print("[yaw_calib] rotating now. Press Enter when the robot reaches the physical target angle.")
    try:
        while time.time() - start_time < timeout_s:
            now = time.time()
            if now - last_command_time >= 0.1:
                driver.send_lr(-spin_speed, spin_speed)
                last_command_time = now
            packet = read_feedback_packet(driver, timeout=0.05)
            if packet is not None:
                latest_packet = packet
                current_deg = compute_delta_yaw_deg(initial_packet, latest_packet, wheel_separation_m, yaw_scale)
                if current_deg is not None:
                    estimated_deg = current_deg
            if select_with_timeout(0.0):
                sys.stdin.readline()
                break
    finally:
        driver.stop()
        time.sleep(0.25)
        driver.stop()

    return {
        "estimated_deg": estimated_deg,
        "duration_s": time.time() - start_time,
        "last_packet": latest_packet,
    }


def select_with_timeout(timeout_s):
    import select
    return bool(select.select([sys.stdin], [], [], timeout_s)[0])


def prompt_yes(stage_deg):
    while True:
        answer = input(f"[yaw_calib] ready for {stage_deg} deg rotation? type y then Enter: ").strip().lower()
        if answer == "y":
            return
        print("[yaw_calib] please type y to start this stage.")


def prompt_actual_deg():
    raw = input("[yaw_calib] measured actual rotation in deg (Enter to skip): ").strip()
    if not raw:
        return None
    return float(raw)


def main():
    parser = argparse.ArgumentParser(description="Interactive yaw calibration using encoder-based spin targets.")
    parser.add_argument("--port", default="", help="Serial port to open. Auto-detect if omitted.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument(
        "--spin-speed",
        type=float,
        default=0.08,
        help="Wheel speed magnitude used for in-place rotation. Use a low value for the first calibration pass.",
    )
    parser.add_argument("--wheel-separation-m", type=float, default=0.52, help="Effective wheel separation used in yaw estimation.")
    parser.add_argument("--yaw-scale", type=float, default=1.0, help="Current wheel yaw scale applied during estimation.")
    parser.add_argument("--timeout", type=float, default=25.0, help="Per-stage timeout in seconds.")
    parser.add_argument("--targets", default="360,360,360", help="Comma-separated target angles in degrees.")
    parser.add_argument(
        "--auto-stop",
        action="store_true",
        help="Use encoder-estimated stop angle automatically. Default is manual-stop mode, which is safer while calibrating.",
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
    output_path = args.output or str(default_dir / f"yaw_calibration_{timestamp}.csv")

    print(f"[yaw_calib] port={port} baud={args.baud} targets={targets} spin_speed={args.spin_speed:.3f}")
    print(
        "[yaw_calib] stop criterion uses encoder-derived yaw from odl/odr with "
        f"wheel_separation_m={args.wheel_separation_m:.3f} and yaw_scale={args.yaw_scale:.3f}"
    )
    if args.auto_stop:
        print("[yaw_calib] mode=auto-stop using encoder-estimated yaw target")
    else:
        print("[yaw_calib] mode=manual-stop; robot spins slowly until you press Enter at the physical target angle")
    print("[yaw_calib] stop ROS stacks first so the serial port is free.")

    driver = BaseDriver(port, args.baud)
    driver.attach_state_store(StateStore())
    rows = []

    try:
        if not args.skip_init:
            for cmd in DEFAULT_BOOT_COMMANDS:
                print(f"[yaw_calib] send {cmd}")
                driver.send_json(cmd)
                time.sleep(0.15)

        warmup_packet = wait_for_encoder_feedback(driver, timeout=6.0)
        if warmup_packet is None:
            latest_packet = driver.get_feedback()
            print("[yaw_calib] no encoder feedback packet received during warmup.")
            print(f"[yaw_calib] latest raw packet seen: {latest_packet!r}")
            raise RuntimeError("No T=1001 feedback with odl/odr during warmup. Check serial port, baud, and that ROS stack is stopped.")
        print(
            "[yaw_calib] warmup feedback ok: T=%s odl=%s odr=%s"
            % (warmup_packet.get("T"), warmup_packet.get("odl"), warmup_packet.get("odr"))
        )

        for target_deg in targets:
            prompt_yes(target_deg)
            if args.auto_stop:
                print(f"[yaw_calib] rotating until estimated yaw reaches {target_deg:.1f} deg")
                result = run_rotation_auto(
                    driver,
                    target_deg=target_deg,
                    spin_speed=args.spin_speed,
                    wheel_separation_m=args.wheel_separation_m,
                    yaw_scale=args.yaw_scale,
                    timeout_s=args.timeout,
                )
            else:
                print(f"[yaw_calib] rotating toward physical target {target_deg:.1f} deg")
                result = run_rotation_manual(
                    driver,
                    spin_speed=args.spin_speed,
                    wheel_separation_m=args.wheel_separation_m,
                    yaw_scale=args.yaw_scale,
                    timeout_s=args.timeout,
                )
            estimated_deg = result["estimated_deg"]
            print(
                "[yaw_calib] stage target=%.1f estimated_stop=%.2f duration=%.2fs"
                % (target_deg, estimated_deg, result["duration_s"])
            )
            if args.auto_stop:
                actual_deg = prompt_actual_deg()
            else:
                actual_raw = input(
                    f"[yaw_calib] actual measured rotation in deg for target {target_deg:.1f} "
                    "(Enter de dung gia tri target): "
                ).strip()
                actual_deg = float(actual_raw) if actual_raw else target_deg
            recommended_scale = None
            if actual_deg is not None and abs(estimated_deg) > 1e-6:
                recommended_scale = args.yaw_scale * (actual_deg / estimated_deg)
                print("[yaw_calib] recommended WHEEL_YAW_SCALE from this stage: %.4f" % recommended_scale)
            input("[yaw_calib] press Enter to continue to the next stage.")
            rows.append(
                {
                    "target_deg": target_deg,
                    "estimated_stop_deg": estimated_deg,
                    "actual_measured_deg": actual_deg if actual_deg is not None else "",
                    "recommended_scale_stage": recommended_scale if recommended_scale is not None else "",
                    "spin_speed": args.spin_speed,
                    "wheel_separation_m": args.wheel_separation_m,
                    "input_yaw_scale": args.yaw_scale,
                    "duration_s": result["duration_s"],
                }
            )

        measured_rows = [row for row in rows if row["actual_measured_deg"] != ""]
        if measured_rows:
            total_actual = sum(float(row["actual_measured_deg"]) for row in measured_rows)
            total_estimated = sum(abs(float(row["estimated_stop_deg"])) for row in measured_rows if abs(float(row["estimated_stop_deg"])) > 1e-6)
            overall_scale = args.yaw_scale * (total_actual / total_estimated)
            print("[yaw_calib] overall recommended WHEEL_YAW_SCALE: %.4f" % overall_scale)
        else:
            overall_scale = ""
            print("[yaw_calib] no measured actual angles entered; no overall scale computed.")

        with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
            fieldnames = [
                "target_deg",
                "estimated_stop_deg",
                "actual_measured_deg",
                "recommended_scale_stage",
                "spin_speed",
                "wheel_separation_m",
                "input_yaw_scale",
                "duration_s",
            ]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        print(f"[yaw_calib] saved results to {output_path}")
        if overall_scale != "":
            print("[yaw_calib] export and retry:")
            print("export WHEEL_YAW_SCALE=%.4f" % overall_scale)
    finally:
        try:
            driver.stop()
            time.sleep(0.2)
            driver.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
