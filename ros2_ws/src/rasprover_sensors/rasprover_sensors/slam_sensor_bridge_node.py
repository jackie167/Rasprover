import json
import math

import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rasprover_msgs.msg import RawRobotFeedback
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf2_ros import TransformBroadcaster


class SlamSensorBridgeNode(Node):
    """Converts raw T=1001 feedback into ROS-standard IMU and odometry topics."""

    def __init__(self):
        super().__init__('slam_sensor_bridge_node')

        self.declare_parameter('feedback_raw_topic', '/robot/state/feedback_raw')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('wheel_odom_topic', '/wheel/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('wheel_separation_m', 0.52)
        self.declare_parameter('gyro_scale', 1.0)
        self.declare_parameter('accel_scale', 1.0)
        self.declare_parameter('mag_scale', 1.0)
        self.declare_parameter('motion_epsilon', 1e-6)

        feedback_raw_topic = self.get_parameter('feedback_raw_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        wheel_odom_topic = self.get_parameter('wheel_odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.wheel_separation = self.get_parameter('wheel_separation_m').get_parameter_value().double_value
        self.gyro_scale = self.get_parameter('gyro_scale').get_parameter_value().double_value
        self.accel_scale = self.get_parameter('accel_scale').get_parameter_value().double_value
        self.mag_scale = self.get_parameter('mag_scale').get_parameter_value().double_value
        self.motion_epsilon = self.get_parameter('motion_epsilon').get_parameter_value().double_value

        self.feedback_sub = self.create_subscription(
            RawRobotFeedback,
            feedback_raw_topic,
            self.handle_raw_feedback,
            20,
        )
        self.imu_pub = self.create_publisher(Imu, imu_topic, 20)
        self.mag_pub = self.create_publisher(MagneticField, mag_topic, 20)
        self.odom_pub = self.create_publisher(Odometry, wheel_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.last_odl = None
        self.last_odr = None
        self.last_stamp_seconds = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info(
            'slam_sensor_bridge_node raw=%s imu=%s mag=%s odom=%s wheel_separation=%.3f'
            % (feedback_raw_topic, imu_topic, mag_topic, wheel_odom_topic, self.wheel_separation)
        )

    @staticmethod
    def _float_value(packet, key, scale=1.0):
        try:
            return float(packet[key]) * scale
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    @staticmethod
    def _yaw_to_quaternion(yaw):
        half_yaw = yaw * 0.5
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

    def handle_raw_feedback(self, msg):
        try:
            packet = json.loads(msg.payload_json) if msg.payload_json else {}
        except json.JSONDecodeError:
            self.get_logger().warning('invalid feedback JSON ignored')
            return

        if not isinstance(packet, dict) or packet.get('T') != 1001:
            return

        self.publish_imu(msg, packet)
        self.publish_mag(msg, packet)
        self.publish_wheel_odom(msg, packet)

    def publish_imu(self, raw_msg, packet):
        gx = self._float_value(packet, 'gx', self.gyro_scale)
        gy = self._float_value(packet, 'gy', self.gyro_scale)
        gz = self._float_value(packet, 'gz', self.gyro_scale)
        ax = self._float_value(packet, 'ax', self.accel_scale)
        ay = self._float_value(packet, 'ay', self.accel_scale)
        az = self._float_value(packet, 'az', self.accel_scale)
        if any(value is None for value in (gx, gy, gz, ax, ay, az)):
            return

        msg = Imu()
        msg.header.stamp = raw_msg.stamp
        msg.header.frame_id = self.imu_frame
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        self.imu_pub.publish(msg)

    def publish_mag(self, raw_msg, packet):
        mx = self._float_value(packet, 'mx', self.mag_scale)
        my = self._float_value(packet, 'my', self.mag_scale)
        mz = self._float_value(packet, 'mz', self.mag_scale)
        if any(value is None for value in (mx, my, mz)):
            return

        msg = MagneticField()
        msg.header.stamp = raw_msg.stamp
        msg.header.frame_id = self.imu_frame
        msg.magnetic_field.x = mx
        msg.magnetic_field.y = my
        msg.magnetic_field.z = mz
        self.mag_pub.publish(msg)

    def publish_wheel_odom(self, raw_msg, packet):
        odl = self._float_value(packet, 'odl')
        odr = self._float_value(packet, 'odr')
        if odl is None or odr is None:
            return

        stamp_seconds = self._stamp_to_seconds(raw_msg.stamp)
        if self.last_odl is None or self.last_odr is None or self.last_stamp_seconds is None:
            self.last_odl = odl
            self.last_odr = odr
            self.last_stamp_seconds = stamp_seconds
            return

        dt = stamp_seconds - self.last_stamp_seconds
        d_left = odl - self.last_odl
        d_right = odr - self.last_odr

        self.last_odl = odl
        self.last_odr = odr
        self.last_stamp_seconds = stamp_seconds

        if dt <= 0.0:
            return

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / max(self.wheel_separation, self.motion_epsilon)
        yaw_mid = self.yaw + 0.5 * d_theta
        self.x += d_center * math.cos(yaw_mid)
        self.y += d_center * math.sin(yaw_mid)
        self.yaw += d_theta

        msg = Odometry()
        msg.header.stamp = raw_msg.stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = d_center / dt
        msg.twist.twist.angular.z = d_theta / dt
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = raw_msg.stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SlamSensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
