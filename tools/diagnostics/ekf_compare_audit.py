#!/usr/bin/env python3

import argparse
import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class OdomState:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    wz: float


class EkfCompareAuditNode(Node):
    def __init__(self, args):
        super().__init__("ekf_compare_audit")
        self.args = args
        self.wheel = None
        self.filtered = None
        self.prev_wheel = None
        self.prev_filtered = None
        self.samples = 0
        self.jump_events = []
        self.max_pos_gap = 0.0
        self.max_yaw_gap_deg = 0.0
        self.last_report_time = 0.0

        self.create_subscription(Odometry, args.wheel_topic, self.handle_wheel, 50)
        self.create_subscription(Odometry, args.filtered_topic, self.handle_filtered, 50)

        self.get_logger().info(
            "compare wheel=%s filtered=%s pos_jump=%.3f yaw_jump_deg=%.1f"
            % (
                args.wheel_topic,
                args.filtered_topic,
                args.position_jump_threshold,
                args.yaw_jump_threshold_deg,
            )
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    @staticmethod
    def to_state(msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        return OdomState(
            stamp=EkfCompareAuditNode.stamp_to_seconds(msg.header.stamp),
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw_from_quaternion(pose.orientation),
            vx=twist.linear.x,
            wz=twist.angular.z,
        )

    def handle_wheel(self, msg):
        self.prev_wheel = self.wheel
        self.wheel = self.to_state(msg)
        self.try_compare()

    def handle_filtered(self, msg):
        self.prev_filtered = self.filtered
        self.filtered = self.to_state(msg)
        self.try_compare()

    def try_compare(self):
        if self.wheel is None or self.filtered is None:
            return
        if abs(self.wheel.stamp - self.filtered.stamp) > self.args.max_time_delta:
            return

        self.samples += 1
        dx = self.filtered.x - self.wheel.x
        dy = self.filtered.y - self.wheel.y
        pos_gap = math.hypot(dx, dy)
        yaw_gap_deg = math.degrees(normalize_angle(self.filtered.yaw - self.wheel.yaw))

        self.max_pos_gap = max(self.max_pos_gap, pos_gap)
        self.max_yaw_gap_deg = max(self.max_yaw_gap_deg, abs(yaw_gap_deg))

        now = max(self.wheel.stamp, self.filtered.stamp)
        if now - self.last_report_time >= self.args.report_period:
            self.last_report_time = now
            self.get_logger().info(
                "samples=%d pos_gap=%.3f yaw_gap_deg=%.2f wheel(vx=%.3f,wz=%.3f) ekf(vx=%.3f,wz=%.3f)"
                % (
                    self.samples,
                    pos_gap,
                    yaw_gap_deg,
                    self.wheel.vx,
                    self.wheel.wz,
                    self.filtered.vx,
                    self.filtered.wz,
                )
            )

        if self.prev_filtered is not None:
            ekf_jump = math.hypot(
                self.filtered.x - self.prev_filtered.x,
                self.filtered.y - self.prev_filtered.y,
            )
            ekf_yaw_jump_deg = abs(
                math.degrees(normalize_angle(self.filtered.yaw - self.prev_filtered.yaw))
            )
            if (
                ekf_jump >= self.args.position_jump_threshold
                or ekf_yaw_jump_deg >= self.args.yaw_jump_threshold_deg
            ):
                event = (
                    now,
                    ekf_jump,
                    ekf_yaw_jump_deg,
                    pos_gap,
                    yaw_gap_deg,
                    self.filtered.vx,
                    self.filtered.wz,
                )
                self.jump_events.append(event)
                self.get_logger().warning(
                    "ekf_jump t=%.3f pos_step=%.3f yaw_step_deg=%.2f pos_gap=%.3f yaw_gap_deg=%.2f ekf(vx=%.3f,wz=%.3f)"
                    % event
                )

    def print_summary(self):
        print("[ekf_compare] summary")
        print(f"  samples: {self.samples}")
        print(f"  max_pos_gap: {self.max_pos_gap:.4f}")
        print(f"  max_yaw_gap_deg: {self.max_yaw_gap_deg:.2f}")
        print(f"  jump_events: {len(self.jump_events)}")
        if self.jump_events:
            first = self.jump_events[0]
            last = self.jump_events[-1]
            print(
                "  first_jump: t=%.3f pos_step=%.3f yaw_step_deg=%.2f pos_gap=%.3f yaw_gap_deg=%.2f ekf_vx=%.3f ekf_wz=%.3f"
                % first
            )
            print(
                "  last_jump: t=%.3f pos_step=%.3f yaw_step_deg=%.2f pos_gap=%.3f yaw_gap_deg=%.2f ekf_vx=%.3f ekf_wz=%.3f"
                % last
            )


def parse_args():
    parser = argparse.ArgumentParser(description="Compare /wheel/odometry against /odometry/filtered and detect EKF jumps.")
    parser.add_argument("--wheel-topic", default="/wheel/odometry")
    parser.add_argument("--filtered-topic", default="/odometry/filtered")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--max-time-delta", type=float, default=0.08)
    parser.add_argument("--position-jump-threshold", type=float, default=0.10)
    parser.add_argument("--yaw-jump-threshold-deg", type=float, default=20.0)
    parser.add_argument("--report-period", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = EkfCompareAuditNode(args)
    end_time = node.get_clock().now().nanoseconds / 1_000_000_000.0 + args.duration
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = node.get_clock().now().nanoseconds / 1_000_000_000.0
            if now >= end_time:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
