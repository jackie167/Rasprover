import glob
import math
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarScanNode(Node):
    """Publish /scan from the onboard serial lidar using the legacy 0x54 frame format."""

    HEADER = 0x54
    FRAME_LEN = 47
    POINTS_PER_FRAME = 12

    def __init__(self):
        super().__init__('lidar_scan_node')

        self.declare_parameter('port', '')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('publish_hz', 12.0)
        self.declare_parameter('debug_log', True)

        configured_port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        self.debug_log = self.get_parameter('debug_log').get_parameter_value().bool_value

        self.port = configured_port or self._detect_port()
        if not self.port:
            raise RuntimeError('No lidar serial device found under /dev/ttyACM* or /dev/ttyUSB*')

        self.ser = serial.Serial(self.port, int(baud), timeout=0.2)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)

        self.angle_bins = [float('inf')] * 360
        self.last_start_angle = None
        self.last_scan_publish_time = time.time()
        self.scan_period = 1.0 / max(publish_hz, 1.0)
        self.bytes_seen = 0
        self.frames_seen = 0
        self.last_debug_log_time = time.time()
        self.logged_first_publish = False

        self.timer = self.create_timer(0.001, self._poll_serial)
        self.get_logger().info(
            'lidar_scan_node port=%s baud=%d frame=%s topic=%s'
            % (self.port, int(baud), self.frame_id, scan_topic)
        )

    @staticmethod
    def _detect_port():
        for pattern in ('/dev/ttyACM*', '/dev/ttyUSB*'):
            matches = sorted(glob.glob(pattern))
            if matches:
                return matches[0]
        return ''

    def _poll_serial(self):
        try:
            if self.ser.in_waiting <= 0:
                self._maybe_log_progress()
                return
            while self.ser.in_waiting > 0:
                header = self.ser.read(1)
                if not header:
                    return
                self.bytes_seen += 1
                if header[0] != self.HEADER:
                    self._maybe_log_progress()
                    continue
                payload = self.ser.read(self.FRAME_LEN - 1)
                if len(payload) != self.FRAME_LEN - 1:
                    return
                self.bytes_seen += len(payload)
                data = header + payload
                self._consume_frame(data)
                self._maybe_log_progress()
        except serial.SerialException as exc:
            self.get_logger().error('lidar serial error: %s' % exc)

    def _consume_frame(self, data):
        start_angle_deg = ((data[5] << 8) | data[4]) * 0.01

        for i in range(self.POINTS_PER_FRAME):
            offset = 6 + i * 3
            distance_mm = (data[offset + 1] << 8) | data[offset]
            distance_m = distance_mm / 1000.0
            angle_deg = (start_angle_deg + i * 0.83333 + 180.0) % 360.0
            bin_idx = int(round(angle_deg)) % 360
            if self.range_min <= distance_m <= self.range_max:
                self.angle_bins[bin_idx] = distance_m

        wrapped = self.last_start_angle is not None and start_angle_deg < self.last_start_angle
        self.last_start_angle = start_angle_deg
        self.frames_seen += 1

        if wrapped and (time.time() - self.last_scan_publish_time) >= 0.5 * self.scan_period:
            self._publish_scan()

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = math.radians(359.0)
        msg.angle_increment = math.radians(1.0)
        msg.time_increment = 0.0
        msg.scan_time = max(time.time() - self.last_scan_publish_time, self.scan_period)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = self.angle_bins[:]
        self.scan_pub.publish(msg)
        if self.debug_log and not self.logged_first_publish:
            valid_points = sum(1 for value in msg.ranges if math.isfinite(value))
            self.get_logger().info(
                'published first /scan with %d valid bins, scan_time=%.3f'
                % (valid_points, msg.scan_time)
            )
            self.logged_first_publish = True

        self.angle_bins = [float('inf')] * 360
        self.last_scan_publish_time = time.time()

    def _maybe_log_progress(self):
        if not self.debug_log:
            return
        now = time.time()
        if now - self.last_debug_log_time < 2.0:
            return
        self.get_logger().info(
            'serial progress in_waiting=%d bytes_seen=%d frames_seen=%d last_start_angle=%s'
            % (
                self.ser.in_waiting,
                self.bytes_seen,
                self.frames_seen,
                'None' if self.last_start_angle is None else f'{self.last_start_angle:.2f}',
            )
        )
        self.last_debug_log_time = now

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
