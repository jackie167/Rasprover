#!/usr/bin/env python3

import argparse
import csv
import glob
import json
import select
import sys
import time
from dataclasses import dataclass

from base_driver import BaseDriver


DEFAULT_BOOT_COMMANDS = (
    {"T": 142, "cmd": 50},
    {"T": 131, "cmd": 1},
    {"T": 143, "cmd": 0},
)

CSV_FIELDS = (
    "wall_time",
    "elapsed",
    "stage",
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
    "raw_json",
)


@dataclass
class Stage:
    name: str
    left: float
    right: float
    note: str


PROFILES = {
    "basic": (
        Stage("idle", 0.0, 0.0, "stationary bias/noise"),
        Stage("forward", 0.20, 0.20, "straight forward"),
        Stage("reverse", -0.20, -0.20, "straight reverse"),
        Stage("spin_left", -0.18, 0.18, "spin in place left"),
        Stage("spin_right", 0.18, -0.18, "spin in place right"),
    ),
    "short": (
        Stage("idle", 0.0, 0.0, "stationary bias/noise"),
        Stage("forward", 0.18, 0.18, "straight forward"),
        Stage("spin_left", -0.16, 0.16, "spin in place left"),
    ),
}


def detect_port():
    for path in ("/dev/serial0", "/dev/ttyAMA0"):
        if glob.glob(path):
            return path
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/serial*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def parse_feedback_line(driver, raw_line):
    if not raw_line:
        return []
    try:
        line = raw_line.decode("utf-8", errors="ignore").strip()
    except Exception:
        return []
    if not line:
        return []

    parsed_objects = []
    search_idx = 0
    decoder = driver.base.json_decoder
    while search_idx < len(line):
        start = line.find("{", search_idx)
        if start == -1:
            break
        try:
            data, end_idx = decoder.raw_decode(line, start)
        except json.JSONDecodeError:
            search_idx = start + 1
            continue
        if isinstance(data, dict):
            parsed_objects.append(data)
        search_idx = end_idx
    return parsed_objects


def feedback_float(packet, key):
    try:
        return float(packet[key])
    except (KeyError, TypeError, ValueError):
        return None


def write_packet_row(writer, packet, stage_name, command_left, command_right, start_time):
    raw_json = json.dumps(packet, ensure_ascii=True, separators=(",", ":"))
    row = {
        "wall_time": time.time(),
        "elapsed": time.time() - start_time,
        "stage": stage_name,
        "command_left": command_left,
        "command_right": command_right,
        "packet_type": packet.get("T"),
        "L": feedback_float(packet, "L"),
        "R": feedback_float(packet, "R"),
        "odl": feedback_float(packet, "odl"),
        "odr": feedback_float(packet, "odr"),
        "gx": feedback_float(packet, "gx"),
        "gy": feedback_float(packet, "gy"),
        "gz": feedback_float(packet, "gz"),
        "ax": feedback_float(packet, "ax"),
        "ay": feedback_float(packet, "ay"),
        "az": feedback_float(packet, "az"),
        "mx": feedback_float(packet, "mx"),
        "my": feedback_float(packet, "my"),
        "mz": feedback_float(packet, "mz"),
        "v": feedback_float(packet, "v"),
        "raw_json": raw_json,
    }
    writer.writerow(row)


def drain_feedback(driver, writer, stage_name, command_left, command_right, start_time):
    wrote_any = False
    while driver.base.rl.s.in_waiting > 0:
        raw_line = driver.base.rl.readline()
        if not raw_line:
            break
        for packet in parse_feedback_line(driver, raw_line):
            if "T" not in packet:
                continue
            write_packet_row(writer, packet, stage_name, command_left, command_right, start_time)
            wrote_any = True
    if wrote_any:
        return

    packet = driver.get_feedback()
    if isinstance(packet, dict) and "T" in packet:
        write_packet_row(writer, packet, stage_name, command_left, command_right, start_time)


def run_stage(driver, writer, stage, sample_period, start_time):
    print("[slam_capture] next stage=%s cmd=(%.3f, %.3f) note=%s" % (stage.name, stage.left, stage.right, stage.note))
    while True:
        answer = input("[slam_capture] ready? type y then Enter to start this stage: ").strip().lower()
        if answer == "y":
            break
        print("[slam_capture] skipped invalid input, please type y to start.")

    print("[slam_capture] stage=%s started. Press Enter to stop this stage." % stage.name)
    driver.send_lr(stage.left, stage.right)
    while True:
        drain_feedback(driver, writer, stage.name, stage.left, stage.right, start_time)
        if select.select([sys.stdin], [], [], 0.0)[0]:
            sys.stdin.readline()
            break
        time.sleep(sample_period)
    driver.stop()
    for _ in range(10):
        drain_feedback(driver, writer, stage.name + "_stop", 0.0, 0.0, start_time)
        time.sleep(sample_period)
    print("[slam_capture] stage=%s stopped" % stage.name)


def main():
    parser = argparse.ArgumentParser(description="Capture IMU/encoder feedback for SLAM preparation.")
    parser.add_argument("--port", default="", help="Serial port to open. Auto-detect if omitted.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--profile", choices=sorted(PROFILES), default="basic", help="Interactive stage profile to run.")
    parser.add_argument("--sample-period", type=float, default=0.02, help="Capture loop period in seconds.")
    parser.add_argument("--output", default="", help="CSV output path. Default is timestamped file in current dir.")
    parser.add_argument("--skip-init", action="store_true", help="Do not send the usual startup commands.")
    args = parser.parse_args()

    port = args.port or detect_port()
    if not port:
        raise SystemExit("No serial device found. Tried /dev/serial0, /dev/ttyAMA0, /dev/ttyUSB*, /dev/ttyACM*.")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_path = args.output or f"slam_capture_{args.profile}_{timestamp}.csv"
    stages = PROFILES[args.profile]

    print("[slam_capture] port=%s baud=%d profile=%s output=%s" % (port, args.baud, args.profile, output_path))
    print("[slam_capture] stages=%s" % ", ".join(stage.name for stage in stages))
    driver = BaseDriver(port, args.baud)
    start_time = time.time()

    try:
        if not args.skip_init:
            for cmd in DEFAULT_BOOT_COMMANDS:
                print("[slam_capture] send %s" % cmd)
                driver.send_json(cmd)
                time.sleep(0.15)

        with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            for stage in stages:
                run_stage(driver, writer, stage, args.sample_period, start_time)

            driver.stop()
            for _ in range(10):
                drain_feedback(driver, writer, "post_stop", 0.0, 0.0, start_time)
                time.sleep(args.sample_period)

        print("[slam_capture] done output=%s" % output_path)
        print("[slam_capture] next: ./ugv-env/bin/python slam_prep_analyze.py --input %s" % output_path)
    finally:
        try:
            driver.stop()
            time.sleep(0.2)
            driver.stop()
        except Exception:
            pass
        try:
            driver.base.gimbal_dev_close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
