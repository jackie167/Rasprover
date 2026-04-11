import json
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node

from rasprover_msgs.msg import CvControlIntent
from rasprover_msgs.msg import CvStatus
from rasprover_msgs.msg import CvTrackingTarget

from .cv_ctrl import OpencvFuncs

FALLBACK_ROOT = Path(__file__).resolve().parents[4]
if str(FALLBACK_ROOT) not in sys.path:
    sys.path.insert(0, str(FALLBACK_ROOT))

from repo_paths import ensure_repo_on_path
from repo_paths import find_repo_root

REPO_ROOT = ensure_repo_on_path(
    find_repo_root(start_path=__file__, fallback_root=FALLBACK_ROOT)
)
from app_config import AppConfig  # noqa: E402


class _DummyReadline:
    def __init__(self):
        self.lidar_angles_show = []
        self.lidar_distances_show = []
        self.sensor_data = []


class CvNodeAdapter:
    """Adapter that turns direct CV actuation into ROS control intents."""

    def __init__(self, node, app_config):
        self.node = node
        self.app_config = app_config
        self.rl = _DummyReadline()
        self.base_light_status = 0
        self.head_light_status = 0
        self.last_json = None
        self.last_tracking_target = None
        self.intent_pub = self.node.create_publisher(CvControlIntent, '/cv/control_intent', 10)
        self.target_pub = self.node.create_publisher(CvTrackingTarget, '/cv/tracking_target', 10)
        self.cvf = None

    def attach_cvf(self, cvf):
        self.cvf = cvf

    def current_mode_name(self):
        if self.cvf is None:
            return 'cv_none'
        return self.node._mode_name(self.cvf.cv_mode)

    def publish_intent(self, kind, **kwargs):
        msg = CvControlIntent()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.source = kwargs.get('source', 'cv_node')
        msg.kind = kind
        msg.mode = kwargs.get('mode', self.current_mode_name())
        msg.target_present = bool(kwargs.get('target_present', True))
        msg.linear = float(kwargs.get('linear', 0.0))
        msg.angular = float(kwargs.get('angular', 0.0))
        msg.pan = float(kwargs.get('pan', 0.0))
        msg.tilt = float(kwargs.get('tilt', 0.0))
        msg.speed = float(kwargs.get('speed', 0.0))
        msg.accel = float(kwargs.get('accel', 0.0))
        msg.base_pwm = int(kwargs.get('base_pwm', 0))
        msg.head_pwm = int(kwargs.get('head_pwm', 0))
        msg.timeout_ms = int(kwargs.get('timeout_ms', 300))
        self.intent_pub.publish(msg)

    def base_json_ctrl(self, payload):
        self.last_json = payload
        if payload.get('T') != self.app_config.cmd('cmd_gimbal_ctrl'):
            return
        self.emit_cv_gimbal_intent(
            pan=payload.get('X', 0.0),
            tilt=payload.get('Y', 0.0),
            speed=payload.get('SPD', 0.0),
            accel=payload.get('ACC', 0.0),
        )

    def lights_ctrl(self, base_pwm, head_pwm):
        self.base_light_status = int(base_pwm)
        self.head_light_status = int(head_pwm)
        self.emit_cv_light_intent(base_pwm=self.base_light_status, head_pwm=self.head_light_status)

    def emit_cv_gimbal_intent(self, pan, tilt, speed, accel, mode=None, timeout_ms=300):
        self.publish_intent(
            'gimbal',
            pan=pan,
            tilt=tilt,
            speed=speed,
            accel=accel,
            mode=mode or self.current_mode_name(),
            timeout_ms=timeout_ms,
        )

    def emit_cv_motion_intent(self, linear, angular, mode=None, timeout_ms=300):
        self.publish_intent(
            'motion',
            linear=linear,
            angular=angular,
            mode=mode or self.current_mode_name(),
            timeout_ms=timeout_ms,
        )

    def emit_cv_light_intent(self, base_pwm, head_pwm, mode=None, timeout_ms=500):
        self.base_light_status = int(base_pwm)
        self.head_light_status = int(head_pwm)
        self.publish_intent(
            'lights',
            base_pwm=self.base_light_status,
            head_pwm=self.head_light_status,
            mode=mode or self.current_mode_name(),
            target_present=bool(self.base_light_status or self.head_light_status),
            timeout_ms=timeout_ms,
        )

    def emit_tracking_target(
        self,
        kind,
        mode,
        frame_width,
        frame_height,
        center_x,
        center_y,
        target_x,
        target_y,
        offset_x,
        offset_y,
        area,
        radius,
        target_present,
    ):
        self.last_tracking_target = {
            'source': 'cv_node',
            'mode': self.node._mode_name(mode),
            'kind': str(kind),
            'target_present': bool(target_present),
            'center_x': float(center_x),
            'center_y': float(center_y),
            'target_x': float(target_x),
            'target_y': float(target_y),
            'offset_x': float(offset_x),
            'offset_y': float(offset_y),
            'area': float(area),
            'radius': float(radius),
            'frame_width': int(frame_width),
            'frame_height': int(frame_height),
        }
        msg = CvTrackingTarget()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.source = 'cv_node'
        msg.mode = self.node._mode_name(mode)
        msg.kind = str(kind)
        msg.target_present = bool(target_present)
        msg.center_x = float(center_x)
        msg.center_y = float(center_y)
        msg.target_x = float(target_x)
        msg.target_y = float(target_y)
        msg.offset_x = float(offset_x)
        msg.offset_y = float(offset_y)
        msg.area = float(area)
        msg.radius = float(radius)
        msg.frame_width = int(frame_width)
        msg.frame_height = int(frame_height)
        self.target_pub.publish(msg)


class CvNode(Node):
    def __init__(self):
        super().__init__('cv_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5051)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.app_config = AppConfig(str(REPO_ROOT / 'config.yaml'))
        self.cv_adapter = CvNodeAdapter(self, self.app_config)
        self.cvf = OpencvFuncs(str(REPO_ROOT), self.cv_adapter)
        self.cv_adapter.attach_cvf(self.cvf)
        self.status_pub = self.create_publisher(CvStatus, '/cv/status', 10)
        self.status_timer = self.create_timer(0.2, self.publish_status)

        self.http_server = None
        self.http_thread = None
        self._start_http_server()
        self.get_logger().info(f'cv_node serving MJPEG on http://{self.host}:{self.port}')

    def _mode_name(self, code_value):
        mapping = {
            self.app_config.code('cv_none'): 'cv_none',
            self.app_config.code('cv_moti'): 'cv_moti',
            self.app_config.code('cv_face'): 'cv_face',
            self.app_config.code('cv_objs'): 'cv_objs',
            self.app_config.code('cv_clor'): 'cv_clor',
            self.app_config.code('mp_hand'): 'mp_hand',
            self.app_config.code('cv_auto'): 'cv_auto',
            self.app_config.code('mp_face'): 'mp_face',
            self.app_config.code('mp_pose'): 'mp_pose',
        }
        return mapping.get(code_value, str(code_value))

    def _reaction_name(self, code_value):
        mapping = {
            self.app_config.code('re_none'): 're_none',
            self.app_config.code('re_capt'): 're_capt',
            self.app_config.code('re_reco'): 're_reco',
        }
        return mapping.get(code_value, str(code_value))

    def publish_status(self):
        target = getattr(self.cv_adapter, 'last_tracking_target', None) or {}
        msg = CvStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = self._mode_name(self.cvf.cv_mode)
        msg.reaction_mode = self._reaction_name(self.cvf.detection_reaction_mode)
        msg.motion_lock = bool(self.cvf.cv_movtion_lock)
        msg.fps = float(self.cvf.video_fps)
        msg.target_present = bool(target.get('target_present', False))
        msg.tracking_state = 'active' if msg.target_present else 'idle'
        msg.target_kind = str(target.get('kind', 'none'))
        msg.target_offset_x = float(target.get('offset_x', 0.0))
        msg.target_offset_y = float(target.get('offset_y', 0.0))
        msg.pan_angle = float(self.cvf.pan_angle)
        msg.tilt_angle = float(self.cvf.tilt_angle)
        msg.led_mode = str(self.cvf.cv_light_mode)
        self.status_pub.publish(msg)

    def frame_generator(self):
        while True:
            frame = self.cvf.frame_process()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def _start_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send_json(self, payload, status=HTTPStatus.OK):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/health':
                    self._send_json({'ok': True, 'node': 'cv_node'})
                    return
                if self.path == '/cv/status':
                    target = getattr(node.cv_adapter, 'last_tracking_target', None) or {}
                    self._send_json({
                        'mode': node._mode_name(node.cvf.cv_mode),
                        'reaction_mode': node._reaction_name(node.cvf.detection_reaction_mode),
                        'motion_lock': bool(node.cvf.cv_movtion_lock),
                        'fps': float(node.cvf.video_fps),
                        'target_present': bool(target.get('target_present', False)),
                        'tracking_state': 'active' if bool(target.get('target_present', False)) else 'idle',
                        'target_kind': str(target.get('kind', 'none')),
                        'target_offset_x': float(target.get('offset_x', 0.0)),
                        'target_offset_y': float(target.get('offset_y', 0.0)),
                        'pan_angle': float(node.cvf.pan_angle),
                        'tilt_angle': float(node.cvf.tilt_angle),
                        'led_mode': str(node.cvf.cv_light_mode),
                    })
                    return
                if self.path == '/cv/tracking_target':
                    target = getattr(node.cv_adapter, 'last_tracking_target', None)
                    if target is None:
                        self._send_json({'target_present': False})
                    else:
                        self._send_json(target)
                    return
                if self.path == '/video_feed':
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    try:
                        for chunk in node.frame_generator():
                            self.wfile.write(chunk)
                    except BrokenPipeError:
                        pass
                    return
                self._send_json({'error': 'not found'}, status=HTTPStatus.NOT_FOUND)

            def do_POST(self):
                if self.path == '/cv/mode':
                    content_length = int(self.headers.get('Content-Length', '0'))
                    body = self.rfile.read(content_length) if content_length else b'{}'
                    try:
                        payload = json.loads(body.decode('utf-8') or '{}')
                    except json.JSONDecodeError:
                        self._send_json({'error': 'invalid json'}, status=HTTPStatus.BAD_REQUEST)
                        return

                    mode_name = str(payload.get('mode', 'cv_none'))
                    motion_lock = payload.get('motion_lock')
                    light_mode = payload.get('light_mode')
                    target_color = payload.get('target_color')

                    mode_value = node.app_config.code(mode_name)
                    if mode_value is None:
                        self._send_json({'error': 'unknown mode'}, status=HTTPStatus.BAD_REQUEST)
                        return

                    node.cvf.set_cv_mode(mode_value)
                    if motion_lock is not None:
                        node.cvf.set_movtion_lock(bool(motion_lock))
                    if light_mode is not None:
                        node.cvf.head_light_ctrl(int(light_mode))
                    if target_color is not None:
                        node.cvf.selet_target_color(str(target_color))
                    self._send_json({
                        'ok': True,
                        'mode': node._mode_name(node.cvf.cv_mode),
                        'motion_lock': bool(node.cvf.cv_movtion_lock),
                        'light_mode': int(node.cvf.cv_light_mode),
                        'target_color': str(target_color) if target_color is not None else 'unchanged',
                    })
                    return

                if self.path == '/cv/test_intent':
                    content_length = int(self.headers.get('Content-Length', '0'))
                    body = self.rfile.read(content_length) if content_length else b'{}'
                    try:
                        payload = json.loads(body.decode('utf-8') or '{}')
                    except json.JSONDecodeError:
                        self._send_json({'error': 'invalid json'}, status=HTTPStatus.BAD_REQUEST)
                        return

                    kind = str(payload.get('kind', 'gimbal'))
                    if kind == 'gimbal':
                        frame_width = int(payload.get('frame_width', 640))
                        frame_height = int(payload.get('frame_height', 480))
                        target_x = float(payload.get('target_x', (frame_width / 2.0) + float(payload.get('pan', 0.0))))
                        target_y = float(payload.get('target_y', (frame_height / 2.0) + float(payload.get('tilt', 0.0))))
                        node.cv_adapter.emit_tracking_target(
                            kind='debug_gimbal',
                            mode=node.cvf.cv_mode,
                            frame_width=frame_width,
                            frame_height=frame_height,
                            center_x=(frame_width / 2.0),
                            center_y=(frame_height / 2.0),
                            target_x=target_x,
                            target_y=target_y,
                            offset_x=(target_x - (frame_width / 2.0)),
                            offset_y=(target_y - (frame_height / 2.0)),
                            area=float(payload.get('area', 0.0)),
                            radius=float(payload.get('radius', 0.0)),
                            target_present=True,
                        )
                        node.cv_adapter.publish_intent(
                            'gimbal',
                            source=str(payload.get('source', 'cv_debug')),
                            mode=str(payload.get('mode', node._mode_name(node.cvf.cv_mode))),
                            pan=float(payload.get('pan', node.cvf.pan_angle)),
                            tilt=float(payload.get('tilt', node.cvf.tilt_angle)),
                            speed=float(payload.get('speed', 15.0)),
                            accel=float(payload.get('accel', 5.0)),
                            target_present=bool(payload.get('target_present', True)),
                            timeout_ms=int(payload.get('timeout_ms', 400)),
                        )
                        self._send_json({'ok': True, 'kind': 'gimbal'})
                        return

                    if kind == 'motion':
                        node.cv_adapter.publish_intent(
                            'motion',
                            source=str(payload.get('source', 'cv_debug')),
                            mode=str(payload.get('mode', node._mode_name(node.cvf.cv_mode))),
                            linear=float(payload.get('linear', 0.0)),
                            angular=float(payload.get('angular', 0.0)),
                            target_present=bool(payload.get('target_present', True)),
                            timeout_ms=int(payload.get('timeout_ms', 300)),
                        )
                        self._send_json({'ok': True, 'kind': 'motion'})
                        return

                    if kind == 'lights':
                        node.cv_adapter.publish_intent(
                            'lights',
                            source=str(payload.get('source', 'cv_debug')),
                            mode=str(payload.get('mode', node._mode_name(node.cvf.cv_mode))),
                            base_pwm=int(payload.get('base_pwm', 0)),
                            head_pwm=int(payload.get('head_pwm', 64)),
                            target_present=bool(payload.get('target_present', True)),
                            timeout_ms=int(payload.get('timeout_ms', 500)),
                        )
                        self._send_json({'ok': True, 'kind': 'lights'})
                        return

                    self._send_json({'error': 'unknown kind'}, status=HTTPStatus.BAD_REQUEST)
                    return

                self._send_json({'error': 'not found'}, status=HTTPStatus.NOT_FOUND)

            def log_message(self, format, *args):
                node.get_logger().debug('http ' + (format % args))

        self.http_server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

    def destroy_node(self):
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()
        try:
            if getattr(self.cvf, 'usb_camera_connected', False):
                self.cvf.camera.release()
            elif hasattr(self.cvf, 'picam2'):
                self.cvf.picam2.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
