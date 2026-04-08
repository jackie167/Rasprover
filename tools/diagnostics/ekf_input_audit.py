#!/usr/bin/env python3

import argparse
import json
import math
import statistics
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rasprover_msgs.msg import RawRobotFeedback
from sensor_msgs.msg import Imu


def sign_label(value, epsilon):
    if value > epsilon:
        return "positive"
    if value < -epsilon:
        return "negative"
    return "near_zero"


def mean_or_zero(values):
    if not values:
        return 0.0
    return statistics.fmean(values)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Sample:
    encoder_dtheta: float
    encoder_turn_rate: float
    imu_gz: float
    odom_angular_z: float
    odom_pose_turn_rate: float
    odom_linear_x: float
    d_left: float
    d_right: float
    dt: float


class EkfInputAuditNode(Node):
    def __init__(self, args):
        super().__init__("ekf_input_audit")
        self.args = args
        self.raw_sub = self.create_subscription(
            RawRobotFeedback,
            args.feedback_topic,
            self.handle_feedback,
            50,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            args.imu_topic,
            self.handle_imu,
            50,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.handle_odom,
            50,
        )

        self.last_feedback_stamp = None
        self.last_odl = None
        self.last_odr = None
        self.latest_imu_gz = None
        self.latest_imu_stamp = None
        self.latest_odom_angular_z = None
        self.latest_odom_linear_x = None
        self.latest_odom_stamp = None
        self.latest_odom_yaw = None
        self.latest_odom_pose_turn_rate = None

        self.samples = []
        self.packet_count = 0
        self.imu_count = 0
        self.odom_count = 0

        self.summary_timer = self.create_timer(args.summary_period, self.print_live_summary)

        self.get_logger().info(
            "audit feedback=%s imu=%s odom=%s wheel_separation=%.3f wheel_yaw_scale=%.3f"
            % (
                args.feedback_topic,
                args.imu_topic,
                args.odom_topic,
                args.wheel_separation,
                args.wheel_yaw_scale,
            )
        )
        self.get_logger().info(
            "expected ROS turning convention: left turn => positive encoder_dtheta, positive imu gz, positive odom angular.z"
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def handle_imu(self, msg):
        self.imu_count += 1
        self.latest_imu_gz = msg.angular_velocity.z
        self.latest_imu_stamp = self.stamp_to_seconds(msg.header.stamp)

    def handle_odom(self, msg):
        self.odom_count += 1
        stamp_seconds = self.stamp_to_seconds(msg.header.stamp)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.latest_odom_yaw is not None and self.latest_odom_stamp is not None:
            odom_dt = stamp_seconds - self.latest_odom_stamp
            if odom_dt > 0.0:
                self.latest_odom_pose_turn_rate = normalize_angle(yaw - self.latest_odom_yaw) / odom_dt
        self.latest_odom_yaw = yaw
        self.latest_odom_angular_z = msg.twist.twist.angular.z
        self.latest_odom_linear_x = msg.twist.twist.linear.x
        self.latest_odom_stamp = stamp_seconds

    def handle_feedback(self, msg):
        self.packet_count += 1
        try:
            packet = json.loads(msg.payload_json) if msg.payload_json else {}
        except json.JSONDecodeError:
            return
        if not isinstance(packet, dict) or packet.get("T") != 1001:
            return

        try:
            odl = float(packet["odl"])
            odr = float(packet["odr"])
            gz = float(packet.get("gz", 0.0))
        except (KeyError, TypeError, ValueError):
            return

        stamp_seconds = self.stamp_to_seconds(msg.stamp)
        if self.last_feedback_stamp is None or self.last_odl is None or self.last_odr is None:
            self.last_feedback_stamp = stamp_seconds
            self.last_odl = odl
            self.last_odr = odr
            return

        dt = stamp_seconds - self.last_feedback_stamp
        d_left = odl - self.last_odl
        d_right = odr - self.last_odr
        self.last_feedback_stamp = stamp_seconds
        self.last_odl = odl
        self.last_odr = odr

        if dt <= 0.0:
            return

        encoder_dtheta = (d_right - d_left) / max(self.args.wheel_separation, 1e-6)
        encoder_dtheta *= self.args.wheel_yaw_scale
        encoder_turn_rate = encoder_dtheta / dt

        imu_gz_for_sample = self.latest_imu_gz if self.latest_imu_gz is not None else gz
        odom_angular_z = self.latest_odom_angular_z if self.latest_odom_angular_z is not None else 0.0
        odom_pose_turn_rate = (
            self.latest_odom_pose_turn_rate if self.latest_odom_pose_turn_rate is not None else 0.0
        )
        odom_linear_x = self.latest_odom_linear_x if self.latest_odom_linear_x is not None else 0.0

        if (
            abs(encoder_turn_rate) < self.args.turn_rate_epsilon
            and abs(imu_gz_for_sample) < self.args.gyro_epsilon
            and abs(odom_angular_z) < self.args.turn_rate_epsilon
            and abs(d_left + d_right) < self.args.motion_epsilon
        ):
            return

        self.samples.append(
            Sample(
                encoder_dtheta=encoder_dtheta,
                encoder_turn_rate=encoder_turn_rate,
                imu_gz=imu_gz_for_sample,
                odom_angular_z=odom_angular_z,
                odom_pose_turn_rate=odom_pose_turn_rate,
                odom_linear_x=odom_linear_x,
                d_left=d_left,
                d_right=d_right,
                dt=dt,
            )
        )

    def print_live_summary(self):
        if not self.samples:
            self.get_logger().info(
                "live packets raw=%d imu=%d odom=%d samples=0"
                % (self.packet_count, self.imu_count, self.odom_count)
            )
            return

        window = self.samples[-min(len(self.samples), 20):]
        avg_encoder = statistics.fmean(sample.encoder_turn_rate for sample in window)
        avg_imu = statistics.fmean(sample.imu_gz for sample in window)
        avg_odom = statistics.fmean(sample.odom_angular_z for sample in window)
        avg_left = statistics.fmean(sample.d_left for sample in window)
        avg_right = statistics.fmean(sample.d_right for sample in window)
        self.get_logger().info(
            "live samples=%d encoder_turn=%.4f imu_gz=%.4f odom_wz=%.4f d_left=%.5f d_right=%.5f"
            % (len(self.samples), avg_encoder, avg_imu, avg_odom, avg_left, avg_right)
        )

    def print_final_summary(self):
        print("[ekf_audit] summary")
        print(f"  raw_packets: {self.packet_count}")
        print(f"  imu_msgs: {self.imu_count}")
        print(f"  odom_msgs: {self.odom_count}")
        print(f"  motion_samples: {len(self.samples)}")

        if not self.samples:
            print("[ekf_audit] no motion samples captured")
            return

        encoder_turn = [sample.encoder_turn_rate for sample in self.samples]
        imu_turn = [sample.imu_gz for sample in self.samples]
        odom_turn = [sample.odom_angular_z for sample in self.samples]
        odom_pose_turn = [sample.odom_pose_turn_rate for sample in self.samples]
        odom_linear = [sample.odom_linear_x for sample in self.samples]

        print(f"  avg_encoder_turn_rate: {statistics.fmean(encoder_turn):.5f}")
        print(f"  avg_imu_gz: {statistics.fmean(imu_turn):.5f}")
        print(f"  avg_odom_angular_z: {statistics.fmean(odom_turn):.5f}")
        print(f"  avg_odom_pose_turn_rate: {statistics.fmean(odom_pose_turn):.5f}")
        print(f"  avg_odom_linear_x: {statistics.fmean(odom_linear):.5f}")

        turn_samples = [
            sample for sample in self.samples
            if abs(sample.encoder_turn_rate) >= self.args.turn_rate_threshold
        ]
        if turn_samples:
            left_like = [sample for sample in turn_samples if sample.encoder_turn_rate > 0.0]
            right_like = [sample for sample in turn_samples if sample.encoder_turn_rate < 0.0]
            print(f"  turn_samples: {len(turn_samples)}")
            print(f"  left_like_samples: {len(left_like)}")
            print(f"  right_like_samples: {len(right_like)}")

            avg_imu_on_turn = statistics.fmean(sample.imu_gz for sample in turn_samples)
            avg_odom_on_turn = statistics.fmean(sample.odom_angular_z for sample in turn_samples)
            avg_pose_turn = statistics.fmean(sample.odom_pose_turn_rate for sample in turn_samples)
            avg_encoder_on_turn = statistics.fmean(sample.encoder_turn_rate for sample in turn_samples)

            print(f"  turn_avg_encoder_turn_rate: {avg_encoder_on_turn:.5f}")
            print(f"  turn_avg_imu_gz: {avg_imu_on_turn:.5f}")
            print(f"  turn_avg_odom_angular_z: {avg_odom_on_turn:.5f}")
            print(f"  turn_avg_odom_pose_turn_rate: {avg_pose_turn:.5f}")
            print(f"  turn_encoder_sign: {sign_label(avg_encoder_on_turn, self.args.turn_rate_epsilon)}")
            print(f"  turn_imu_sign: {sign_label(avg_imu_on_turn, self.args.gyro_epsilon)}")
            print(f"  turn_odom_sign: {sign_label(avg_odom_on_turn, self.args.turn_rate_epsilon)}")
            print(f"  turn_odom_pose_sign: {sign_label(avg_pose_turn, self.args.turn_rate_epsilon)}")

            if sign_label(avg_encoder_on_turn, self.args.turn_rate_epsilon) == sign_label(avg_imu_on_turn, self.args.gyro_epsilon):
                print("  imu_vs_encoder_sign: match")
            else:
                print("  imu_vs_encoder_sign: mismatch")

            if sign_label(avg_encoder_on_turn, self.args.turn_rate_epsilon) == sign_label(avg_odom_on_turn, self.args.turn_rate_epsilon):
                print("  odom_vs_encoder_sign: match")
            else:
                print("  odom_vs_encoder_sign: mismatch")

            self.print_phase_summary("left_turn_only", left_like)
            self.print_phase_summary("right_turn_only", right_like)

        forward_samples = [
            sample for sample in self.samples
            if abs(sample.d_left + sample.d_right) >= self.args.motion_threshold
        ]
        if forward_samples:
            avg_d_left = statistics.fmean(sample.d_left for sample in forward_samples)
            avg_d_right = statistics.fmean(sample.d_right for sample in forward_samples)
            avg_linear_x = statistics.fmean(sample.odom_linear_x for sample in forward_samples)
            print(f"  motion_avg_d_left: {avg_d_left:.6f}")
            print(f"  motion_avg_d_right: {avg_d_right:.6f}")
            print(f"  motion_avg_odom_linear_x: {avg_linear_x:.5f}")
            print(f"  motion_linear_x_sign: {sign_label(avg_linear_x, self.args.motion_epsilon)}")

        forward_only = [
            sample for sample in self.samples
            if sample.odom_linear_x >= self.args.linear_threshold
            and abs(sample.encoder_turn_rate) < self.args.turn_rate_threshold
        ]
        self.print_phase_summary("forward_only", forward_only)

        print("[ekf_audit] expected convention")
        print("  left turn => encoder_turn_rate > 0, imu_gz > 0, odom_angular_z > 0")
        print("  forward drive => odom_linear_x > 0")

    def print_phase_summary(self, label, samples):
        print(f"[ekf_audit] phase {label}")
        print(f"  samples: {len(samples)}")
        if not samples:
            return

        avg_encoder = mean_or_zero([sample.encoder_turn_rate for sample in samples])
        avg_imu = mean_or_zero([sample.imu_gz for sample in samples])
        avg_odom = mean_or_zero([sample.odom_angular_z for sample in samples])
        avg_pose = mean_or_zero([sample.odom_pose_turn_rate for sample in samples])
        avg_linear = mean_or_zero([sample.odom_linear_x for sample in samples])
        avg_left = mean_or_zero([sample.d_left for sample in samples])
        avg_right = mean_or_zero([sample.d_right for sample in samples])

        print(f"  avg_encoder_turn_rate: {avg_encoder:.5f}")
        print(f"  avg_imu_gz: {avg_imu:.5f}")
        print(f"  avg_odom_angular_z: {avg_odom:.5f}")
        print(f"  avg_odom_pose_turn_rate: {avg_pose:.5f}")
        print(f"  avg_odom_linear_x: {avg_linear:.5f}")
        print(f"  avg_d_left: {avg_left:.6f}")
        print(f"  avg_d_right: {avg_right:.6f}")
        print(f"  encoder_sign: {sign_label(avg_encoder, self.args.turn_rate_epsilon)}")
        print(f"  imu_sign: {sign_label(avg_imu, self.args.gyro_epsilon)}")
        print(f"  odom_sign: {sign_label(avg_odom, self.args.turn_rate_epsilon)}")
        print(f"  odom_pose_sign: {sign_label(avg_pose, self.args.turn_rate_epsilon)}")
        print(f"  linear_x_sign: {sign_label(avg_linear, self.args.linear_epsilon)}")


def parse_args():
    parser = argparse.ArgumentParser(description="Audit raw EKF inputs for sign and axis consistency.")
    parser.add_argument("--feedback-topic", default="/robot/state/feedback_raw")
    parser.add_argument("--imu-topic", default="/imu/data_raw")
    parser.add_argument("--odom-topic", default="/wheel/odometry")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--summary-period", type=float, default=1.0)
    parser.add_argument("--wheel-separation", type=float, default=0.52)
    parser.add_argument("--wheel-yaw-scale", type=float, default=2.80)
    parser.add_argument("--turn-rate-threshold", type=float, default=0.03)
    parser.add_argument("--turn-rate-epsilon", type=float, default=0.01)
    parser.add_argument("--gyro-epsilon", type=float, default=0.01)
    parser.add_argument("--linear-threshold", type=float, default=0.03)
    parser.add_argument("--linear-epsilon", type=float, default=0.01)
    parser.add_argument("--motion-threshold", type=float, default=0.002)
    parser.add_argument("--motion-epsilon", type=float, default=1e-6)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = EkfInputAuditNode(args)
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
        node.print_final_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
