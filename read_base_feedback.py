#!/usr/bin/env python3

import argparse
import glob
import json
import time

from base_driver import BaseDriver


DEFAULT_BOOT_COMMANDS = (
    {"T": 142, "cmd": 50},
    {"T": 131, "cmd": 1},
    {"T": 143, "cmd": 0},
)

CANDIDATE_TOKENS = (
    "speed_left",
    "speed_right",
    "velocity",
    "vel",
    "pwm",
    "left",
    "right",
    "encoder",
    "odom",
    "imu",
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


def extract_candidates(packet):
    if not isinstance(packet, dict):
        return {}
    return {
        key: value
        for key, value in packet.items()
        if any(token in str(key).lower() for token in CANDIDATE_TOKENS)
    }


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


def main():
    parser = argparse.ArgumentParser(description="Read and print raw feedback packets from the robot base.")
    parser.add_argument("--port", default="", help="Serial port to open. Auto-detect if omitted.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--duration", type=float, default=15.0, help="Seconds to read feedback.")
    parser.add_argument(
        "--skip-init",
        action="store_true",
        help="Do not send the usual startup commands (T=142, T=131, T=143).",
    )
    args = parser.parse_args()

    port = args.port or detect_port()
    if not port:
        raise SystemExit("No serial device found. Tried /dev/serial0, /dev/ttyAMA0, /dev/ttyUSB*, /dev/ttyACM*.")

    print(f"[feedback_reader] port={port} baud={args.baud} duration={args.duration}")
    driver = BaseDriver(port, args.baud)

    try:
        if not args.skip_init:
            for cmd in DEFAULT_BOOT_COMMANDS:
                print(f"[feedback_reader] send {cmd}")
                driver.send_json(cmd)
                time.sleep(0.1)

        end_time = time.time() + args.duration
        while time.time() < end_time:
            saw_raw = False
            while driver.base.rl.s.in_waiting > 0:
                raw_line = driver.base.rl.readline()
                if not raw_line:
                    break
                saw_raw = True
                print(f"[feedback_reader] raw={raw_line!r}")
                packets = parse_feedback_line(driver, raw_line)
                for packet in packets:
                    print(f"[feedback_reader] latest={packet}")
                    candidates = extract_candidates(packet)
                    if candidates:
                        print(f"[feedback_reader] candidates={candidates}")
                    driver.base.base_data = packet

            if not saw_raw:
                packet = driver.get_feedback()
                if isinstance(packet, dict):
                    print(f"[feedback_reader] latest={packet}")
                    candidates = extract_candidates(packet)
                    if candidates:
                        print(f"[feedback_reader] candidates={candidates}")
            time.sleep(0.1)
    finally:
        try:
            driver.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
