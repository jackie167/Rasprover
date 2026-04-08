import math

import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


class SimpleOdomFilterNode(Node):
    def __init__(self):
        super().__init__("simple_odom_filter_node")

        self.input_odom_topic = self.declare_parameter("input_odom_topic", "/wheel/odometry").value
        self.output_odom_topic = self.declare_parameter("output_odom_topic", "/odometry/filtered").value
        self.imu_topic = self.declare_parameter("imu_topic", "/imu/data_raw").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.use_imu = bool(self.declare_parameter("use_imu", False).value)
        self.linear_alpha = float(self.declare_parameter("linear_alpha", 0.35).value)
        self.angular_alpha = float(self.declare_parameter("angular_alpha", 0.35).value)
        self.imu_angular_alpha = float(self.declare_parameter("imu_angular_alpha", 0.25).value)
        self.imu_angular_weight = float(self.declare_parameter("imu_angular_weight", 0.0).value)
        self.max_linear_velocity = float(self.declare_parameter("max_linear_velocity", 1.0).value)
        self.max_angular_velocity = float(self.declare_parameter("max_angular_velocity", 3.0).value)
        self.max_linear_accel = float(self.declare_parameter("max_linear_accel", 1.5).value)
        self.max_angular_accel = float(self.declare_parameter("max_angular_accel", 6.0).value)
        self.pose_correction_gain_xy = float(self.declare_parameter("pose_correction_gain_xy", 0.10).value)
        self.pose_correction_gain_yaw = float(self.declare_parameter("pose_correction_gain_yaw", 0.10).value)
        self.max_pose_correction_step = float(self.declare_parameter("max_pose_correction_step", 0.03).value)
        self.max_yaw_correction_step = float(self.declare_parameter("max_yaw_correction_step", 0.08).value)
        self.pose_xy_covariance = float(self.declare_parameter("pose_xy_covariance", 0.20).value)
        self.pose_yaw_covariance = float(self.declare_parameter("pose_yaw_covariance", 0.12).value)
        self.twist_linear_covariance = float(self.declare_parameter("twist_linear_covariance", 0.08).value)
        self.twist_yaw_covariance = float(self.declare_parameter("twist_yaw_covariance", 0.10).value)

        self.odom_sub = self.create_subscription(Odometry, self.input_odom_topic, self.handle_wheel_odom, 50)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.handle_imu, 50) if self.use_imu else None
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.last_stamp = None
        self.filtered_vx = 0.0
        self.filtered_wz = 0.0
        self.latest_imu_wz = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.initialized_pose = False

        self.get_logger().info(
            "simple_odom_filter_node input=%s output=%s publish_tf=%s use_imu=%s linear_alpha=%.2f angular_alpha=%.2f imu_weight=%.2f"
            % (
                self.input_odom_topic,
                self.output_odom_topic,
                self.publish_tf,
                self.use_imu,
                self.linear_alpha,
                self.angular_alpha,
                self.imu_angular_weight,
            )
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def handle_imu(self, msg):
        raw_wz = msg.angular_velocity.z
        raw_wz = clamp(raw_wz, -self.max_angular_velocity, self.max_angular_velocity)
        if self.latest_imu_wz is None:
            self.latest_imu_wz = raw_wz
            return
        self.latest_imu_wz = (
            self.imu_angular_alpha * raw_wz
            + (1.0 - self.imu_angular_alpha) * self.latest_imu_wz
        )

    def handle_wheel_odom(self, msg):
        stamp_seconds = self.stamp_to_seconds(msg.header.stamp)
        wheel_pose = msg.pose.pose
        wheel_yaw = yaw_from_quaternion(wheel_pose.orientation)
        raw_vx = clamp(msg.twist.twist.linear.x, -self.max_linear_velocity, self.max_linear_velocity)
        raw_wz = clamp(msg.twist.twist.angular.z, -self.max_angular_velocity, self.max_angular_velocity)

        if not self.initialized_pose:
            self.x = wheel_pose.position.x
            self.y = wheel_pose.position.y
            self.yaw = wheel_yaw
            self.filtered_vx = raw_vx
            self.filtered_wz = raw_wz
            self.last_stamp = stamp_seconds
            self.initialized_pose = True
            self.publish_filtered(msg.header.stamp)
            return

        dt = stamp_seconds - self.last_stamp if self.last_stamp is not None else 0.0
        self.last_stamp = stamp_seconds
        if dt <= 0.0:
            return

        linear_delta_limit = self.max_linear_accel * dt
        angular_delta_limit = self.max_angular_accel * dt

        linear_target = self.linear_alpha * raw_vx + (1.0 - self.linear_alpha) * self.filtered_vx
        angular_target = self.angular_alpha * raw_wz + (1.0 - self.angular_alpha) * self.filtered_wz

        if self.use_imu and self.latest_imu_wz is not None:
            angular_target = (
                (1.0 - self.imu_angular_weight) * angular_target
                + self.imu_angular_weight * self.latest_imu_wz
            )

        self.filtered_vx += clamp(linear_target - self.filtered_vx, -linear_delta_limit, linear_delta_limit)
        self.filtered_wz += clamp(angular_target - self.filtered_wz, -angular_delta_limit, angular_delta_limit)
        self.filtered_vx = clamp(self.filtered_vx, -self.max_linear_velocity, self.max_linear_velocity)
        self.filtered_wz = clamp(self.filtered_wz, -self.max_angular_velocity, self.max_angular_velocity)

        yaw_mid = self.yaw + 0.5 * self.filtered_wz * dt
        self.x += self.filtered_vx * dt * math.cos(yaw_mid)
        self.y += self.filtered_vx * dt * math.sin(yaw_mid)
        self.yaw = normalize_angle(self.yaw + self.filtered_wz * dt)

        dx_err = wheel_pose.position.x - self.x
        dy_err = wheel_pose.position.y - self.y
        yaw_err = normalize_angle(wheel_yaw - self.yaw)

        self.x += clamp(dx_err * self.pose_correction_gain_xy, -self.max_pose_correction_step, self.max_pose_correction_step)
        self.y += clamp(dy_err * self.pose_correction_gain_xy, -self.max_pose_correction_step, self.max_pose_correction_step)
        self.yaw = normalize_angle(
            self.yaw + clamp(yaw_err * self.pose_correction_gain_yaw, -self.max_yaw_correction_step, self.max_yaw_correction_step)
        )

        self.publish_filtered(msg.header.stamp)

    def publish_filtered(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        msg.pose.covariance[0] = self.pose_xy_covariance
        msg.pose.covariance[7] = self.pose_xy_covariance
        msg.pose.covariance[14] = 1e6
        msg.pose.covariance[21] = 1e6
        msg.pose.covariance[28] = 1e6
        msg.pose.covariance[35] = self.pose_yaw_covariance
        msg.twist.twist.linear.x = self.filtered_vx
        msg.twist.twist.angular.z = self.filtered_wz
        msg.twist.covariance[0] = self.twist_linear_covariance
        msg.twist.covariance[7] = 1e6
        msg.twist.covariance[14] = 1e6
        msg.twist.covariance[21] = 1e6
        msg.twist.covariance[28] = 1e6
        msg.twist.covariance[35] = self.twist_yaw_covariance
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
