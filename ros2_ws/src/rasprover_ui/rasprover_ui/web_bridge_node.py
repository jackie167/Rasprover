import json
import mimetypes
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.error import URLError
from urllib.parse import unquote, urlparse
from urllib.request import Request
from urllib.request import urlopen

try:
    from . import audio_support as audio_ctrl
except Exception:  # pragma: no cover
    audio_ctrl = None
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rasprover_msgs.msg import GimbalCommand
from rasprover_msgs.msg import LightCommand
from rasprover_msgs.msg import MotionCommand
from rasprover_msgs.msg import RobotFeedback
from rasprover_msgs.msg import ServoSetupCommand


REPO_ROOT = Path(__file__).resolve().parents[4]


def find_repo_root():
    env_root = os.environ.get('PROJECT_DIR')
    if env_root:
        candidate = Path(env_root).resolve()
        if (candidate / 'config.yaml').exists():
            return candidate

    for candidate in Path(__file__).resolve().parents:
        if (candidate / 'config.yaml').exists() and (candidate / 'templates').exists():
            return candidate

    cwd_candidate = Path.cwd().resolve()
    if (cwd_candidate / 'config.yaml').exists():
        return cwd_candidate

    return REPO_ROOT


def build_motion_command(node, payload):
    msg = MotionCommand()
    msg.stamp = node.get_clock().now().to_msg()
    msg.source = str(payload.get('source', 'web_http'))
    msg.linear = float(payload.get('linear', 0.0))
    msg.angular = float(payload.get('angular', 0.0))
    msg.priority = int(payload.get('priority', 1))
    msg.mode = str(payload.get('mode', 'manual'))
    msg.timeout_ms = int(payload.get('timeout_ms', 500))
    msg.frame_id = str(payload.get('frame_id', 'base_link'))
    return msg


def build_gimbal_command(node, payload):
    msg = GimbalCommand()
    msg.stamp = node.get_clock().now().to_msg()
    msg.source = str(payload.get('source', 'web_http'))
    msg.pan = float(payload.get('pan', 0.0))
    msg.tilt = float(payload.get('tilt', 0.0))
    msg.speed = float(payload.get('speed', 0.0))
    msg.accel = float(payload.get('accel', 0.0))
    msg.mode = str(payload.get('mode', 'manual'))
    msg.timeout_ms = int(payload.get('timeout_ms', 1000))
    return msg


def build_light_command(node, payload):
    msg = LightCommand()
    msg.stamp = node.get_clock().now().to_msg()
    msg.source = str(payload.get('source', 'web_http'))
    msg.mode = str(payload.get('mode', 'manual'))
    msg.base_pwm = int(payload.get('base_pwm', 0))
    msg.head_pwm = int(payload.get('head_pwm', 0))
    msg.timeout_ms = int(payload.get('timeout_ms', 2000))
    return msg


def build_servo_setup_command(node, payload):
    msg = ServoSetupCommand()
    msg.stamp = node.get_clock().now().to_msg()
    msg.source = str(payload.get('source', 'web_http'))
    msg.action = str(payload.get('action', ''))
    msg.old_id = int(payload.get('old_id', 0))
    msg.new_id = int(payload.get('new_id', 0))
    msg.servo_id = int(payload.get('servo_id', 0))
    msg.status = int(payload.get('status', 0))
    return msg


class WebBridgeNode(Node):
    """Thin HTTP bridge from browser requests into ROS topics."""

    def __init__(self):
        super().__init__('web_bridge_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5050)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.cv_host = '127.0.0.1'
        self.cv_port = 5051
        self.repo_root = find_repo_root()
        self.config_path = self.repo_root / 'config.yaml'
        self.photo_dir = self.repo_root / 'templates' / 'pictures'
        self.video_dir = self.repo_root / 'templates' / 'videos'
        self.audio_dir = self.repo_root / 'sounds' / 'others'

        self.latest_feedback = {
            'battery_voltage': 0.0,
            'motion_state': 'unknown',
            'gimbal_pan': 0.0,
            'gimbal_tilt': 0.0,
            'base_light_pwm': 0,
            'head_light_pwm': 0,
            'fault_flags': [],
            'raw_available': False,
        }
        self._feedback_lock = threading.Lock()

        self.motion_pub = self.create_publisher(MotionCommand, '/ui/cmd/motion', 10)
        self.gimbal_pub = self.create_publisher(GimbalCommand, '/ui/cmd/gimbal', 10)
        self.light_pub = self.create_publisher(LightCommand, '/ui/cmd/lights', 10)
        self.servo_setup_pub = self.create_publisher(ServoSetupCommand, '/ui/cmd/servo_setup', 10)
        self.stop_pub = self.create_publisher(Bool, '/system/cmd/stop', 10)
        self.feedback_sub = self.create_subscription(
            RobotFeedback,
            '/robot/state/feedback',
            self.handle_feedback,
            10,
        )

        self.http_server = None
        self.http_thread = None
        self._start_http_server()
        self.get_logger().info(f'web_bridge_node serving HTTP on http://{self.host}:{self.port}')

    @staticmethod
    def _sorted_files(directory, suffixes=None):
        if not directory.exists():
            return []
        files = []
        for path in directory.iterdir():
            if not path.is_file():
                continue
            if suffixes and path.suffix.lower() not in suffixes:
                continue
            files.append(path)
        files.sort(key=lambda item: item.stat().st_mtime, reverse=True)
        return [path.name for path in files]

    @staticmethod
    def _safe_child(directory, filename):
        target = (directory / filename).resolve()
        if directory.resolve() not in target.parents and target != directory.resolve():
            raise ValueError('invalid path')
        return target

    def _start_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send_bytes(self, body, content_type, status=HTTPStatus.OK):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def _send_json(self, payload, status=HTTPStatus.OK):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self):
                content_length = int(self.headers.get('Content-Length', '0') or 0)
                if content_length <= 0:
                    return {}
                data = self.rfile.read(content_length)
                return json.loads(data.decode('utf-8'))

            def do_OPTIONS(self):
                self._send_json({'ok': True})

            def do_GET(self):
                parsed = urlparse(self.path)
                route = parsed.path

                if route == '/health':
                    self._send_json({'ok': True, 'node': 'web_bridge_node'})
                    return
                if route == '/state':
                    with node._feedback_lock:
                        payload = dict(node.latest_feedback)
                    self._send_json(payload)
                    return
                if route == '/api/cv/status':
                    payload, status = node.cv_request('GET', '/cv/status')
                    self._send_json(payload, status=status)
                    return
                if route == '/api/cv/tracking_target':
                    payload, status = node.cv_request('GET', '/cv/tracking_target')
                    self._send_json(payload, status=status)
                    return
                if route == '/api/config/raw':
                    self._send_bytes(node.config_path.read_bytes(), 'text/plain; charset=utf-8')
                    return
                if route == '/photo':
                    self._send_bytes(node.render_photo_page().encode('utf-8'), 'text/html; charset=utf-8')
                    return
                if route == '/video':
                    self._send_bytes(node.render_video_page().encode('utf-8'), 'text/html; charset=utf-8')
                    return
                if route == '/settings':
                    self._send_bytes(node.render_settings_page().encode('utf-8'), 'text/html; charset=utf-8')
                    return
                if route == '/legacy-ui':
                    self._send_bytes(node.render_legacy_ui_page().encode('utf-8'), 'text/html; charset=utf-8')
                    return
                if route == '/api/photos':
                    self._send_json(node._sorted_files(node.photo_dir, {'.jpg', '.jpeg', '.png'}))
                    return
                if route == '/api/videos':
                    self._send_json(node._sorted_files(node.video_dir, {'.mp4', '.mov', '.avi'}))
                    return
                if route == '/api/audio':
                    self._send_json(node._sorted_files(node.audio_dir, {'.mp3', '.wav'}))
                    return
                if route.startswith('/photos/'):
                    filename = unquote(route[len('/photos/'):])
                    try:
                        target = node._safe_child(node.photo_dir, filename)
                        body = target.read_bytes()
                    except Exception:
                        self._send_json({'error': 'not found'}, status=HTTPStatus.NOT_FOUND)
                        return
                    content_type = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
                    self._send_bytes(body, content_type)
                    return
                if route.startswith('/videos/'):
                    filename = unquote(route[len('/videos/'):])
                    try:
                        target = node._safe_child(node.video_dir, filename)
                        body = target.read_bytes()
                    except Exception:
                        self._send_json({'error': 'not found'}, status=HTTPStatus.NOT_FOUND)
                        return
                    content_type = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
                    self._send_bytes(body, content_type)
                    return
                if route == '/':
                    html = node.render_index_html().encode('utf-8')
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(html)))
                    self.end_headers()
                    self.wfile.write(html)
                    return
                self._send_json({'error': 'not found'}, status=HTTPStatus.NOT_FOUND)

            def do_POST(self):
                parsed = urlparse(self.path)
                route = parsed.path
                try:
                    payload = self._read_json()
                except json.JSONDecodeError:
                    self._send_json({'error': 'invalid json'}, status=HTTPStatus.BAD_REQUEST)
                    return

                if route == '/api/motion':
                    msg = build_motion_command(node, payload)
                    node.motion_pub.publish(msg)
                    node.get_logger().info(
                        'web published motion source=%s linear=%.3f angular=%.3f'
                        % (msg.source, msg.linear, msg.angular)
                    )
                    self._send_json({'ok': True})
                    return

                if route == '/api/gimbal':
                    msg = build_gimbal_command(node, payload)
                    node.gimbal_pub.publish(msg)
                    node.get_logger().info(
                        'web published gimbal source=%s pan=%.3f tilt=%.3f'
                        % (msg.source, msg.pan, msg.tilt)
                    )
                    self._send_json({'ok': True})
                    return

                if route == '/api/lights':
                    msg = build_light_command(node, payload)
                    node.light_pub.publish(msg)
                    node.get_logger().info(
                        'web published lights source=%s base_pwm=%d head_pwm=%d'
                        % (msg.source, msg.base_pwm, msg.head_pwm)
                    )
                    self._send_json({'ok': True})
                    return

                if route == '/api/settings/servo':
                    msg = build_servo_setup_command(node, payload)
                    node.servo_setup_pub.publish(msg)
                    node.get_logger().info(
                        'web published servo_setup source=%s action=%s'
                        % (msg.source, msg.action)
                    )
                    self._send_json({'ok': True})
                    return

                if route == '/api/stop':
                    stop_msg = Bool()
                    stop_msg.data = bool(payload.get('active', True))
                    node.stop_pub.publish(stop_msg)
                    node.get_logger().info('web published stop=%s' % stop_msg.data)
                    self._send_json({'ok': True, 'active': stop_msg.data})
                    return

                if route == '/api/cv/mode':
                    response_payload, status = node.cv_request('POST', '/cv/mode', payload)
                    self._send_json(response_payload, status=status)
                    return

                if route == '/api/cv/test_intent':
                    response_payload, status = node.cv_request('POST', '/cv/test_intent', payload)
                    self._send_json(response_payload, status=status)
                    return

                if route == '/api/audio/play':
                    if audio_ctrl is None:
                        self._send_json({'ok': False, 'error': 'audio unavailable'}, status=HTTPStatus.SERVICE_UNAVAILABLE)
                        return
                    audio_file = str(payload.get('audio_file', ''))
                    try:
                        target = node._safe_child(node.audio_dir, audio_file)
                    except Exception:
                        self._send_json({'error': 'invalid file'}, status=HTTPStatus.BAD_REQUEST)
                        return
                    audio_ctrl.play_audio_thread(str(target))
                    self._send_json({'ok': True})
                    return

                if route == '/api/audio/stop':
                    if audio_ctrl is None:
                        self._send_json({'ok': False, 'error': 'audio unavailable'}, status=HTTPStatus.SERVICE_UNAVAILABLE)
                        return
                    audio_ctrl.stop()
                    self._send_json({'ok': True})
                    return

                if route == '/api/photos/delete':
                    filename = str(payload.get('filename', ''))
                    try:
                        target = node._safe_child(node.photo_dir, filename)
                        os.remove(target)
                        self._send_json({'ok': True})
                    except Exception:
                        self._send_json({'ok': False}, status=HTTPStatus.BAD_REQUEST)
                    return

                if route == '/api/videos/delete':
                    filename = str(payload.get('filename', ''))
                    try:
                        target = node._safe_child(node.video_dir, filename)
                        os.remove(target)
                        self._send_json({'ok': True})
                    except Exception:
                        self._send_json({'ok': False}, status=HTTPStatus.BAD_REQUEST)
                    return

                self._send_json({'error': 'not found'}, status=HTTPStatus.NOT_FOUND)

            def log_message(self, format, *args):
                node.get_logger().debug('http ' + (format % args))

        self.http_server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

    def handle_feedback(self, msg):
        with self._feedback_lock:
            self.latest_feedback = {
                'battery_voltage': float(msg.battery_voltage),
                'motion_state': str(msg.motion_state),
                'gimbal_pan': float(msg.gimbal_pan),
                'gimbal_tilt': float(msg.gimbal_tilt),
                'base_light_pwm': int(msg.base_light_pwm),
                'head_light_pwm': int(msg.head_light_pwm),
                'fault_flags': list(msg.fault_flags),
                'raw_available': bool(msg.raw_available),
            }

    def cv_request(self, method, path, payload=None):
        url = f'http://{self.cv_host}:{self.cv_port}{path}'
        body = None
        headers = {}
        if payload is not None:
            body = json.dumps(payload).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        request = Request(url, data=body, headers=headers, method=method)
        try:
            with urlopen(request, timeout=3) as response:
                raw = response.read()
                data = json.loads(raw.decode('utf-8') or '{}')
                return data, HTTPStatus(response.status)
        except HTTPError as exc:
            raw = exc.read()
            try:
                data = json.loads(raw.decode('utf-8') or '{}')
            except Exception:
                data = {'error': 'cv node http error'}
            return data, HTTPStatus(exc.code)
        except URLError:
            return {'error': 'cv node unavailable'}, HTTPStatus.SERVICE_UNAVAILABLE

    def render_index_html(self):
        return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rasprover Motion UI</title>
  <style>
    :root {
      --bg: #f3efe6;
      --panel: rgba(255,255,255,0.9);
      --ink: #15231a;
      --accent: #d66a2a;
      --accent-dark: #8c3d12;
      --line: rgba(21,35,26,0.12);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Trebuchet MS", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(214,106,42,0.18), transparent 30%),
        radial-gradient(circle at bottom right, rgba(38,92,66,0.18), transparent 28%),
        linear-gradient(135deg, #efe7d7, #f8f5ee 55%, #e6efe6);
      min-height: 100vh;
    }
    .shell {
      max-width: 980px;
      margin: 0 auto;
      padding: 28px 18px 40px;
    }
    .hero {
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 20px;
    }
    .hero h1 {
      margin: 0;
      font-size: clamp(2rem, 4vw, 3rem);
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }
    .hero p {
      margin: 6px 0 0;
      opacity: 0.72;
      max-width: 520px;
    }
    .badge {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 10px 14px;
      font-weight: 700;
      box-shadow: 0 10px 25px rgba(21,35,26,0.08);
    }
    .grid {
      display: grid;
      grid-template-columns: 1.2fr 0.8fr;
      gap: 18px;
    }
    .nav-links {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 14px;
    }
    .nav-links a {
      display: inline-block;
      padding: 8px 12px;
      border-radius: 999px;
      background: rgba(53, 86, 71, 0.12);
      color: var(--ink);
      text-decoration: none;
      font-weight: 700;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 24px;
      padding: 20px;
      box-shadow: 0 16px 40px rgba(21,35,26,0.08);
      backdrop-filter: blur(10px);
    }
    .card h2 {
      margin: 0 0 14px;
      font-size: 1rem;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, minmax(90px, 1fr));
      gap: 12px;
      margin-top: 18px;
    }
    .mini-pad {
      display: grid;
      grid-template-columns: repeat(3, minmax(72px, 1fr));
      gap: 10px;
      margin-top: 14px;
    }
    .ctrl {
      border: 0;
      border-radius: 18px;
      padding: 18px 14px;
      font-size: 1rem;
      font-weight: 700;
      cursor: pointer;
      color: white;
      background: linear-gradient(180deg, var(--accent), var(--accent-dark));
      box-shadow: 0 10px 18px rgba(140,61,18,0.22);
      transition: transform 0.08s ease, box-shadow 0.08s ease;
    }
    .ctrl:active {
      transform: translateY(2px);
      box-shadow: 0 6px 12px rgba(140,61,18,0.18);
    }
    .ctrl.secondary {
      background: linear-gradient(180deg, #4d7260, #274539);
      box-shadow: 0 10px 18px rgba(39,69,57,0.22);
    }
    .ctrl.stop {
      background: linear-gradient(180deg, #cb4040, #791c1c);
      box-shadow: 0 10px 18px rgba(121,28,28,0.22);
    }
    .empty { visibility: hidden; }
    .stat {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      padding: 12px 0;
      border-bottom: 1px solid var(--line);
    }
    .stat:last-child { border-bottom: 0; }
    .label {
      font-size: 0.84rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      opacity: 0.7;
    }
    .value {
      font-size: 1.2rem;
      font-weight: 800;
    }
    .sliders {
      display: grid;
      gap: 16px;
      margin-top: 18px;
    }
    .triple {
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }
    .slider-label {
      display: flex;
      justify-content: space-between;
      font-weight: 700;
      margin-bottom: 8px;
    }
    input[type=range] {
      width: 100%;
      accent-color: var(--accent);
    }
    .foot {
      margin-top: 18px;
      opacity: 0.7;
      font-size: 0.9rem;
    }
    .media-list {
      display: grid;
      gap: 10px;
      max-height: 260px;
      overflow: auto;
      margin-top: 12px;
    }
    .media-item {
      display: flex;
      gap: 8px;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid var(--line);
      padding-bottom: 8px;
    }
    .media-item a {
      color: var(--accent-dark);
      text-decoration: none;
      font-weight: 700;
      word-break: break-all;
    }
    .mini-actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }
    .small-btn {
      border: 0;
      border-radius: 12px;
      padding: 8px 10px;
      cursor: pointer;
      color: white;
      background: #355647;
      font-weight: 700;
    }
    @media (max-width: 760px) {
      .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="hero">
      <div>
        <h1>Rasprover Motion</h1>
        <p>ROS-native motion UI. This page talks to <code>web_bridge_node</code>, which forwards motion into <code>command_mux_node</code> and then <code>robot_base_node</code>.</p>
        <div class="nav-links">
          <a href="/photo">Photos</a>
          <a href="/video">Videos</a>
          <a href="/settings">Settings</a>
          <a href="/legacy-ui">Legacy Style</a>
          <a href="#" id="cvFeedLink" target="_blank">CV Feed</a>
        </div>
      </div>
      <div class="badge" id="healthBadge">Bridge Ready</div>
    </div>

    <div class="grid">
      <div class="card">
        <h2>Drive Pad</h2>
        <div class="sliders">
          <div>
            <div class="slider-label"><span>Linear Speed</span><span id="linearOut">0.35</span></div>
            <input id="linearScale" type="range" min="0.1" max="1.0" step="0.05" value="0.35">
          </div>
          <div>
            <div class="slider-label"><span>Angular Speed</span><span id="angularOut">0.60</span></div>
            <input id="angularScale" type="range" min="0.1" max="1.5" step="0.05" value="0.60">
          </div>
        </div>

        <div class="pad">
          <div class="empty ctrl"></div>
          <button class="ctrl" data-linear="1" data-angular="0">Forward</button>
          <div class="empty ctrl"></div>
          <button class="ctrl secondary" data-linear="0" data-angular="1">Left</button>
          <button class="ctrl stop" id="stopBtn">Stop</button>
          <button class="ctrl secondary" data-linear="0" data-angular="-1">Right</button>
          <div class="empty ctrl"></div>
          <button class="ctrl" data-linear="-1" data-angular="0">Reverse</button>
          <div class="empty ctrl"></div>
        </div>

        <div class="foot">
          Keyboard: <strong>W/A/S/D</strong> for motion, <strong>Space</strong> for stop.
        </div>
      </div>

      <div class="card">
        <h2>Gimbal</h2>
        <div class="sliders">
          <div>
            <div class="slider-label"><span>Pan</span><span id="panTargetOut">0</span></div>
            <input id="panTarget" type="range" min="-180" max="180" step="1" value="0">
          </div>
          <div>
            <div class="slider-label"><span>Tilt</span><span id="tiltTargetOut">0</span></div>
            <input id="tiltTarget" type="range" min="-90" max="90" step="1" value="0">
          </div>
          <div class="sliders triple">
            <div>
              <div class="slider-label"><span>Speed</span><span id="gimbalSpeedOut">0</span></div>
              <input id="gimbalSpeed" type="range" min="0" max="100" step="1" value="0">
            </div>
            <div>
              <div class="slider-label"><span>Accel</span><span id="gimbalAccelOut">0</span></div>
              <input id="gimbalAccel" type="range" min="0" max="100" step="1" value="0">
            </div>
          </div>
        </div>
        <div class="mini-pad">
          <button class="ctrl secondary" id="sendGimbalBtn">Send</button>
          <button class="ctrl secondary" id="centerGimbalBtn">Center</button>
          <button class="ctrl stop" id="stopGimbalBtn">Stop</button>
        </div>
      </div>

      <div class="card">
        <h2>Lights</h2>
        <div class="sliders">
          <div>
            <div class="slider-label"><span>Base PWM</span><span id="baseLightTargetOut">0</span></div>
            <input id="baseLightTarget" type="range" min="0" max="255" step="1" value="0">
          </div>
          <div>
            <div class="slider-label"><span>Head PWM</span><span id="headLightTargetOut">0</span></div>
            <input id="headLightTarget" type="range" min="0" max="255" step="1" value="0">
          </div>
        </div>
        <div class="mini-pad">
          <button class="ctrl secondary" id="applyLightsBtn">Apply</button>
          <button class="ctrl secondary" id="allLightsOnBtn">All On</button>
          <button class="ctrl stop" id="allLightsOffBtn">All Off</button>
        </div>
      </div>

      <div class="card">
        <h2>Robot State</h2>
        <div class="stat"><span class="label">Battery</span><span class="value" id="batteryValue">--</span></div>
        <div class="stat"><span class="label">Motion</span><span class="value" id="motionValue">--</span></div>
        <div class="stat"><span class="label">Gimbal Pan</span><span class="value" id="panValue">--</span></div>
        <div class="stat"><span class="label">Gimbal Tilt</span><span class="value" id="tiltValue">--</span></div>
        <div class="stat"><span class="label">Base Light</span><span class="value" id="baseLightValue">--</span></div>
        <div class="stat"><span class="label">Head Light</span><span class="value" id="headLightValue">--</span></div>
        <div class="stat"><span class="label">Faults</span><span class="value" id="faultValue">none</span></div>
      </div>

      <div class="card">
        <h2>CV Control</h2>
        <div class="mini-actions">
          <button class="small-btn" id="cvNoneBtn">CV Off</button>
          <button class="small-btn" id="cvFaceBtn">Face Track</button>
          <button class="small-btn" id="cvColorBtn">Color Track</button>
          <button class="small-btn" id="cvAutoBtn">Auto Drive</button>
        </div>
        <div class="mini-actions" style="margin-top: 10px;">
          <button class="small-btn" id="cvObjectsBtn">Objects</button>
          <button class="small-btn" id="cvHandBtn">Hand</button>
          <button class="small-btn" id="cvMpFaceBtn">MP Face</button>
          <button class="small-btn" id="cvPoseBtn">MP Pose</button>
        </div>
        <div class="mini-actions" style="margin-top: 10px;">
          <button class="small-btn" id="cvLightOffBtn">Light Off</button>
          <button class="small-btn" id="cvLightOnBtn">Light On</button>
          <button class="small-btn" id="refreshCvBtn">Refresh CV</button>
        </div>
        <div class="mini-actions" style="margin-top: 10px;">
          <button class="small-btn" id="cvTrackOnBtn">Track On</button>
          <button class="small-btn" id="cvTrackOffBtn">Track Off</button>
        </div>
        <div class="mini-actions" style="margin-top: 10px;">
          <button class="small-btn" id="cvColorRedBtn">Red</button>
          <button class="small-btn" id="cvColorGreenBtn">Green</button>
          <button class="small-btn" id="cvColorBlueBtn">Blue</button>
        </div>
        <div class="mini-actions" style="margin-top: 10px;">
          <button class="small-btn" id="cvTestGimbalBtn">Test Gimbal Intent</button>
          <button class="small-btn" id="cvTestLightBtn">Test Light Intent</button>
          <button class="small-btn" id="cvTestMotionBtn">Test Motion Intent</button>
        </div>
        <div class="stat"><span class="label">CV Mode</span><span class="value" id="cvModeValue">--</span></div>
        <div class="stat"><span class="label">Tracking</span><span class="value" id="cvTrackingValue">--</span></div>
        <div class="stat"><span class="label">CV FPS</span><span class="value" id="cvFpsValue">--</span></div>
        <div class="stat"><span class="label">CV Pan</span><span class="value" id="cvPanValue">--</span></div>
        <div class="stat"><span class="label">CV Tilt</span><span class="value" id="cvTiltValue">--</span></div>
        <div class="stat"><span class="label">Target Kind</span><span class="value" id="cvTargetKindValue">--</span></div>
        <div class="stat"><span class="label">Target Offset</span><span class="value" id="cvTargetOffsetValue">--</span></div>
        <div class="stat"><span class="label">Target XY</span><span class="value" id="cvTargetPointValue">--</span></div>
        <div class="stat"><span class="label">Frame</span><span class="value" id="cvFrameValue">--</span></div>
      </div>

      <div class="card">
        <h2>Media</h2>
        <div class="mini-actions">
          <button class="small-btn" id="refreshMediaBtn">Refresh</button>
          <button class="small-btn" id="playFirstAudioBtn">Play First Audio</button>
          <button class="small-btn" id="stopAudioBtn">Stop Audio</button>
        </div>
        <h3>Photos</h3>
        <div class="media-list" id="photoList"></div>
        <h3>Videos</h3>
        <div class="media-list" id="videoList"></div>
        <h3>Audio</h3>
        <div class="media-list" id="audioList"></div>
      </div>
    </div>
  </div>

  <script>
    const linearScale = document.getElementById('linearScale');
    const angularScale = document.getElementById('angularScale');
    const linearOut = document.getElementById('linearOut');
    const angularOut = document.getElementById('angularOut');
    const panTarget = document.getElementById('panTarget');
    const tiltTarget = document.getElementById('tiltTarget');
    const gimbalSpeed = document.getElementById('gimbalSpeed');
    const gimbalAccel = document.getElementById('gimbalAccel');
    const baseLightTarget = document.getElementById('baseLightTarget');
    const headLightTarget = document.getElementById('headLightTarget');
    const panTargetOut = document.getElementById('panTargetOut');
    const tiltTargetOut = document.getElementById('tiltTargetOut');
    const gimbalSpeedOut = document.getElementById('gimbalSpeedOut');
    const gimbalAccelOut = document.getElementById('gimbalAccelOut');
    const baseLightTargetOut = document.getElementById('baseLightTargetOut');
    const headLightTargetOut = document.getElementById('headLightTargetOut');
    const photoList = document.getElementById('photoList');
    const videoList = document.getElementById('videoList');
    const audioList = document.getElementById('audioList');
    const cvFeedLink = document.getElementById('cvFeedLink');
    const healthBadge = document.getElementById('healthBadge');
    const cvModeValue = document.getElementById('cvModeValue');
    const cvTrackingValue = document.getElementById('cvTrackingValue');
    const cvFpsValue = document.getElementById('cvFpsValue');
    const cvPanValue = document.getElementById('cvPanValue');
    const cvTiltValue = document.getElementById('cvTiltValue');
    const cvTargetKindValue = document.getElementById('cvTargetKindValue');
    const cvTargetOffsetValue = document.getElementById('cvTargetOffsetValue');
    const cvTargetPointValue = document.getElementById('cvTargetPointValue');
    const cvFrameValue = document.getElementById('cvFrameValue');

    cvFeedLink.href = `http://${window.location.hostname}:5051/video_feed`;

    function updateScaleLabels() {
      linearOut.textContent = Number(linearScale.value).toFixed(2);
      angularOut.textContent = Number(angularScale.value).toFixed(2);
      panTargetOut.textContent = String(panTarget.value);
      tiltTargetOut.textContent = String(tiltTarget.value);
      gimbalSpeedOut.textContent = String(gimbalSpeed.value);
      gimbalAccelOut.textContent = String(gimbalAccel.value);
      baseLightTargetOut.textContent = String(baseLightTarget.value);
      headLightTargetOut.textContent = String(headLightTarget.value);
    }

    linearScale.addEventListener('input', updateScaleLabels);
    angularScale.addEventListener('input', updateScaleLabels);
    panTarget.addEventListener('input', updateScaleLabels);
    tiltTarget.addEventListener('input', updateScaleLabels);
    gimbalSpeed.addEventListener('input', updateScaleLabels);
    gimbalAccel.addEventListener('input', updateScaleLabels);
    baseLightTarget.addEventListener('input', updateScaleLabels);
    headLightTarget.addEventListener('input', updateScaleLabels);
    updateScaleLabels();

    async function postJson(path, payload) {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      return response.json();
    }

    async function getJson(path) {
      const response = await fetch(path);
      return response.json();
    }

    async function sendMotion(linearSign, angularSign, source = 'web_bridge_ui') {
      const linear = linearSign * Number(linearScale.value);
      const angular = angularSign * Number(angularScale.value);
      await postJson('/api/motion', {
        source,
        linear,
        angular,
        priority: 1,
        mode: 'manual',
        timeout_ms: 500,
        frame_id: 'base_link'
      });
    }

    async function sendStop() {
      await postJson('/api/stop', { active: true });
      setTimeout(() => postJson('/api/stop', { active: false }), 120);
    }

    async function sendGimbal(source = 'web_bridge_ui') {
      await postJson('/api/gimbal', {
        source,
        pan: Number(panTarget.value),
        tilt: Number(tiltTarget.value),
        speed: Number(gimbalSpeed.value),
        accel: Number(gimbalAccel.value),
        mode: 'manual',
        timeout_ms: 1000
      });
    }

    async function sendLights(source = 'web_bridge_ui') {
      await postJson('/api/lights', {
        source,
        mode: 'manual',
        base_pwm: Number(baseLightTarget.value),
        head_pwm: Number(headLightTarget.value),
        timeout_ms: 2000
      });
    }

    async function setCvMode(mode, options = {}) {
      const payload = { mode };
      if (Object.prototype.hasOwnProperty.call(options, 'light_mode')) payload.light_mode = options.light_mode;
      if (Object.prototype.hasOwnProperty.call(options, 'motion_lock')) payload.motion_lock = options.motion_lock;
      if (Object.prototype.hasOwnProperty.call(options, 'target_color')) payload.target_color = options.target_color;
      return postJson('/api/cv/mode', payload);
    }

    async function sendCvTestIntent(kind) {
      const payload = {
        kind,
        source: 'web_cv_debug',
        mode: kind === 'gimbal' ? 'cv_face' : 'cv_none',
        timeout_ms: kind === 'gimbal' ? 400 : 500
      };
      if (kind === 'gimbal') {
        payload.pan = Number(panTarget.value);
        payload.tilt = Number(tiltTarget.value);
        payload.speed = Number(gimbalSpeed.value || 15);
        payload.accel = Number(gimbalAccel.value || 5);
      } else if (kind === 'motion') {
        payload.linear = Number(linearScale.value) * 0.5;
        payload.angular = Number(angularScale.value) * -0.5;
      } else {
        payload.base_pwm = Number(baseLightTarget.value);
        payload.head_pwm = Number(headLightTarget.value || 64);
      }
      return postJson('/api/cv/test_intent', payload);
    }

    function renderFileList(container, items, basePath, deletePath) {
      container.innerHTML = '';
      if (!items.length) {
        container.innerHTML = '<div class="label">No files</div>';
        return;
      }
      items.forEach(name => {
        const row = document.createElement('div');
        row.className = 'media-item';
        const link = document.createElement('a');
        link.href = basePath + encodeURIComponent(name);
        link.target = '_blank';
        link.textContent = name;
        row.appendChild(link);
        if (deletePath) {
          const del = document.createElement('button');
          del.className = 'small-btn';
          del.textContent = 'Delete';
          del.addEventListener('click', async () => {
            await postJson(deletePath, { filename: name });
            await refreshMedia();
          });
          row.appendChild(del);
        }
        container.appendChild(row);
      });
    }

    function renderAudioList(items) {
      audioList.innerHTML = '';
      if (!items.length) {
        audioList.innerHTML = '<div class="label">No audio files</div>';
        return;
      }
      items.forEach(name => {
        const row = document.createElement('div');
        row.className = 'media-item';
        const label = document.createElement('span');
        label.textContent = name;
        row.appendChild(label);
        const play = document.createElement('button');
        play.className = 'small-btn';
        play.textContent = 'Play';
        play.addEventListener('click', async () => {
          await postJson('/api/audio/play', { audio_file: name });
        });
        row.appendChild(play);
        audioList.appendChild(row);
      });
    }

    async function refreshMedia() {
      try {
        const [photos, videos, audio] = await Promise.all([
          getJson('/api/photos'),
          getJson('/api/videos'),
          getJson('/api/audio')
        ]);
        renderFileList(photoList, photos, '/photos/', '/api/photos/delete');
        renderFileList(videoList, videos, '/videos/', '/api/videos/delete');
        renderAudioList(audio);
      } catch (error) {
        photoList.innerHTML = '<div class="label">Media unavailable</div>';
        videoList.innerHTML = '<div class="label">Media unavailable</div>';
        audioList.innerHTML = '<div class="label">Media unavailable</div>';
      }
    }

    async function refreshCv() {
      try {
        const [state, target] = await Promise.all([
          getJson('/api/cv/status'),
          getJson('/api/cv/tracking_target')
        ]);
        cvModeValue.textContent = state.mode;
        cvTrackingValue.textContent = state.motion_lock ? 'locked' : 'tracking';
        cvFpsValue.textContent = Number(state.fps).toFixed(1);
        cvPanValue.textContent = Number(state.pan_angle).toFixed(1);
        cvTiltValue.textContent = Number(state.tilt_angle).toFixed(1);
        cvTargetKindValue.textContent = state.target_present ? `${state.target_kind} @ ${state.mode}` : 'none';
        cvTargetOffsetValue.textContent = state.target_present
          ? `${Number(state.target_offset_x).toFixed(1)}, ${Number(state.target_offset_y).toFixed(1)}`
          : '--';
        cvTargetPointValue.textContent = target.target_present
          ? `${Number(target.target_x).toFixed(1)}, ${Number(target.target_y).toFixed(1)}`
          : '--';
        cvFrameValue.textContent = target.target_present
          ? `${target.frame_width} x ${target.frame_height}`
          : '--';
      } catch (error) {
        cvModeValue.textContent = 'offline';
        cvTrackingValue.textContent = '--';
        cvFpsValue.textContent = '--';
        cvPanValue.textContent = '--';
        cvTiltValue.textContent = '--';
        cvTargetKindValue.textContent = '--';
        cvTargetOffsetValue.textContent = '--';
        cvTargetPointValue.textContent = '--';
        cvFrameValue.textContent = '--';
      }
    }

    document.querySelectorAll('button[data-linear]').forEach(button => {
      button.addEventListener('click', async () => {
        await sendMotion(Number(button.dataset.linear), Number(button.dataset.angular));
      });
    });

    document.getElementById('stopBtn').addEventListener('click', sendStop);
    document.getElementById('sendGimbalBtn').addEventListener('click', () => sendGimbal());
    document.getElementById('centerGimbalBtn').addEventListener('click', async () => {
      panTarget.value = '0';
      tiltTarget.value = '0';
      updateScaleLabels();
      await sendGimbal('web_bridge_center');
    });
    document.getElementById('stopGimbalBtn').addEventListener('click', async () => {
      panTarget.value = '0';
      tiltTarget.value = '0';
      gimbalSpeed.value = '0';
      gimbalAccel.value = '0';
      updateScaleLabels();
      await sendGimbal('web_bridge_gimbal_stop');
    });
    document.getElementById('applyLightsBtn').addEventListener('click', () => sendLights());
    document.getElementById('allLightsOnBtn').addEventListener('click', async () => {
      baseLightTarget.value = '255';
      headLightTarget.value = '255';
      updateScaleLabels();
      await sendLights('web_bridge_lights_on');
    });
    document.getElementById('allLightsOffBtn').addEventListener('click', async () => {
      baseLightTarget.value = '0';
      headLightTarget.value = '0';
      updateScaleLabels();
      await sendLights('web_bridge_lights_off');
    });
    document.getElementById('refreshMediaBtn').addEventListener('click', refreshMedia);
    document.getElementById('cvNoneBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', { light_mode: 0, motion_lock: true });
      await refreshCv();
    });
    document.getElementById('cvFaceBtn').addEventListener('click', async () => {
      await setCvMode('cv_face', { light_mode: 1, motion_lock: false });
      await refreshCv();
    });
    document.getElementById('cvColorBtn').addEventListener('click', async () => {
      await setCvMode('cv_clor', { light_mode: 1, motion_lock: false });
      await refreshCv();
    });
    document.getElementById('cvAutoBtn').addEventListener('click', async () => {
      await setCvMode('cv_auto', { light_mode: 0, motion_lock: false });
      await refreshCv();
    });
    document.getElementById('cvObjectsBtn').addEventListener('click', async () => {
      await setCvMode('cv_objs', { light_mode: 0, motion_lock: true });
      await refreshCv();
    });
    document.getElementById('cvHandBtn').addEventListener('click', async () => {
      await setCvMode('mp_hand', { light_mode: 0, motion_lock: false });
      await refreshCv();
    });
    document.getElementById('cvMpFaceBtn').addEventListener('click', async () => {
      await setCvMode('mp_face', { light_mode: 0, motion_lock: true });
      await refreshCv();
    });
    document.getElementById('cvPoseBtn').addEventListener('click', async () => {
      await setCvMode('mp_pose', { light_mode: 0, motion_lock: true });
      await refreshCv();
    });
    document.getElementById('cvLightOffBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', { light_mode: 0 });
      await refreshCv();
    });
    document.getElementById('cvLightOnBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', { light_mode: 2 });
      await refreshCv();
    });
    document.getElementById('cvTrackOnBtn').addEventListener('click', async () => {
      await setCvMode(cvModeValue.textContent === 'cv_none' ? 'cv_face' : cvModeValue.textContent, { motion_lock: false });
      await refreshCv();
    });
    document.getElementById('cvTrackOffBtn').addEventListener('click', async () => {
      await setCvMode(cvModeValue.textContent === 'cv_none' ? 'cv_face' : cvModeValue.textContent, { motion_lock: true });
      await refreshCv();
    });
    document.getElementById('cvColorRedBtn').addEventListener('click', async () => {
      await setCvMode('cv_clor', { target_color: 'red', motion_lock: false, light_mode: 1 });
      await refreshCv();
    });
    document.getElementById('cvColorGreenBtn').addEventListener('click', async () => {
      await setCvMode('cv_clor', { target_color: 'green', motion_lock: false, light_mode: 1 });
      await refreshCv();
    });
    document.getElementById('cvColorBlueBtn').addEventListener('click', async () => {
      await setCvMode('cv_clor', { target_color: 'blue', motion_lock: false, light_mode: 1 });
      await refreshCv();
    });
    document.getElementById('refreshCvBtn').addEventListener('click', refreshCv);
    document.getElementById('cvTestGimbalBtn').addEventListener('click', async () => {
      await sendCvTestIntent('gimbal');
      await refreshCv();
    });
    document.getElementById('cvTestLightBtn').addEventListener('click', async () => {
      await sendCvTestIntent('lights');
      await refreshCv();
    });
    document.getElementById('cvTestMotionBtn').addEventListener('click', async () => {
      await sendCvTestIntent('motion');
      await refreshCv();
    });
    document.getElementById('playFirstAudioBtn').addEventListener('click', async () => {
      const files = await getJson('/api/audio');
      if (files.length) {
        await postJson('/api/audio/play', { audio_file: files[0] });
      }
    });
    document.getElementById('stopAudioBtn').addEventListener('click', async () => {
      await postJson('/api/audio/stop', {});
    });

    document.addEventListener('keydown', async (event) => {
      if (event.repeat) return;
      if (event.code === 'KeyW') await sendMotion(1, 0, 'keyboard');
      if (event.code === 'KeyS') await sendMotion(-1, 0, 'keyboard');
      if (event.code === 'KeyA') await sendMotion(0, 1, 'keyboard');
      if (event.code === 'KeyD') await sendMotion(0, -1, 'keyboard');
      if (event.code === 'Space') await sendStop();
    });

    async function refreshState() {
      try {
        const state = await fetch('/state').then(r => r.json());
        document.getElementById('batteryValue').textContent = state.battery_voltage.toFixed(2) + ' V';
        document.getElementById('motionValue').textContent = state.motion_state;
        document.getElementById('panValue').textContent = state.gimbal_pan.toFixed(1);
        document.getElementById('tiltValue').textContent = state.gimbal_tilt.toFixed(1);
        document.getElementById('baseLightValue').textContent = String(state.base_light_pwm);
        document.getElementById('headLightValue').textContent = String(state.head_light_pwm);
        baseLightTarget.value = String(state.base_light_pwm);
        headLightTarget.value = String(state.head_light_pwm);
        baseLightTargetOut.textContent = String(state.base_light_pwm);
        headLightTargetOut.textContent = String(state.head_light_pwm);
        document.getElementById('faultValue').textContent = state.fault_flags.length ? state.fault_flags.join(', ') : 'none';
        healthBadge.textContent = state.raw_available ? 'Bridge Online' : 'Waiting Feedback';
      } catch (error) {
        healthBadge.textContent = 'Bridge Offline';
      }
    }

    refreshState();
    refreshMedia();
    refreshCv();
    setInterval(refreshState, 500);
    setInterval(refreshCv, 1000);
  </script>
</body>
</html>
"""

    def render_photo_page(self):
        return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rasprover Photos</title>
  <style>
    body { font-family: "Trebuchet MS", sans-serif; margin: 0; background: #f7f3ea; color: #1c2b20; }
    .wrap { max-width: 980px; margin: 0 auto; padding: 24px; }
    .top { display: flex; gap: 12px; flex-wrap: wrap; align-items: center; justify-content: space-between; }
    .btn { display: inline-block; padding: 10px 14px; border-radius: 14px; background: #355647; color: #fff; text-decoration: none; border: 0; cursor: pointer; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 16px; margin-top: 20px; }
    .card { background: #fff; border-radius: 18px; padding: 12px; box-shadow: 0 10px 24px rgba(0,0,0,0.08); }
    img { width: 100%; height: 180px; object-fit: cover; border-radius: 12px; background: #eee; }
    .meta { margin-top: 10px; word-break: break-all; font-weight: 700; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <h1>Photo Gallery</h1>
      <div>
        <a class="btn" href="/">Home</a>
        <a class="btn" href="/video">Videos</a>
      </div>
    </div>
    <div id="count"></div>
    <div class="grid" id="grid"></div>
  </div>
  <script>
    async function loadPhotos() {
      const photos = await fetch('/api/photos').then(r => r.json());
      document.getElementById('count').textContent = `Total: ${photos.length}`;
      const grid = document.getElementById('grid');
      grid.innerHTML = '';
      for (const name of photos) {
        const card = document.createElement('div');
        card.className = 'card';
        card.innerHTML = `
          <a href="/photos/${encodeURIComponent(name)}" target="_blank"><img src="/photos/${encodeURIComponent(name)}" alt="${name}"></a>
          <div class="meta">${name}</div>
          <button class="btn">Delete</button>
        `;
        card.querySelector('button').addEventListener('click', async () => {
          await fetch('/api/photos/delete', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ filename: name }) });
          loadPhotos();
        });
        grid.appendChild(card);
      }
    }
    loadPhotos();
  </script>
</body>
</html>
"""

    def render_legacy_ui_page(self):
        return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rasprover Legacy Style</title>
  <style>
    :root {
      --bg: #1e1e20;
      --panel: rgba(28, 30, 34, 0.98);
      --line: rgba(202, 213, 255, 0.1);
      --ink: #d8d8d8;
      --accent: #4ff5c0;
      --soft: rgba(203, 213, 255, 0.08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Trebuchet MS", "Segoe UI", sans-serif;
      color: var(--ink);
      background: var(--bg);
      min-height: 100vh;
    }
    .shell { max-width: 1380px; margin: 0 auto; padding: 22px 18px 30px; }
    .hero {
      display: flex;
      gap: 18px;
      flex-wrap: wrap;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 18px;
    }
    .hero h1 {
      margin: 0;
      font-size: 2.4rem;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }
    .hero p {
      margin: 8px 0 0;
      opacity: 0.72;
      max-width: 760px;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(720px, 780px) minmax(460px, 1fr);
      gap: 18px;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 18px;
      box-shadow: 0 12px 28px rgba(0,0,0,0.25);
    }
    .links {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 12px;
    }
    .links a, .links button {
      display: inline-block;
      padding: 8px 12px;
      border-radius: 8px;
      border: 1px solid var(--line);
      background: var(--soft);
      color: var(--ink);
      text-decoration: none;
      font-weight: 700;
      cursor: pointer;
    }
    .video-shell {
      position: relative;
      width: 100%;
      aspect-ratio: 4 / 3;
      overflow: hidden;
      border-radius: 8px;
      background: #000;
      border: 1px solid rgba(255,255,255,0.05);
    }
    .video-shell img {
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: block;
    }
    .overlay {
      position: absolute;
      inset: 18px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      pointer-events: none;
      color: var(--accent);
      font-weight: 700;
      text-shadow: 0 1px 2px rgba(0,0,0,0.5);
    }
    .overlay-top,
    .overlay-bottom {
      display: flex;
      justify-content: space-between;
      gap: 16px;
    }
    .overlay-center {
      display: flex;
      justify-content: space-between;
      align-items: center;
      min-height: 120px;
    }
    .focus-box {
      width: 110px;
      height: 22px;
      border-top: 2px solid var(--accent);
      border-bottom: 2px solid var(--accent);
      opacity: 0.75;
    }
    .stack p {
      margin: 0;
      line-height: 1.5;
      font-size: 0.95rem;
    }
    .status-pill {
      border: 1px solid var(--accent);
      border-radius: 8px;
      padding: 4px 8px;
      display: inline-flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 6px;
      font-size: 0.95rem;
    }
    .badge-dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      background: var(--accent);
      box-shadow: 0 0 12px rgba(79,245,192,0.5);
    }
    .video-actions {
      margin-top: 14px;
      display: flex;
      gap: 10px;
      justify-content: center;
      flex-wrap: wrap;
    }
    .btn {
      border: 1px solid var(--line);
      background: var(--soft);
      color: var(--ink);
      border-radius: 8px;
      min-width: 120px;
      height: 46px;
      padding: 0 14px;
      cursor: pointer;
      font-weight: 700;
    }
    .btn:hover, .links a:hover, .links button:hover { color: var(--accent); }
    .capture-btn {
      width: 68px;
      min-width: 68px;
      height: 68px;
      border-radius: 999px;
      font-size: 0;
      position: relative;
    }
    .capture-btn::after {
      content: "";
      position: absolute;
      inset: 18px;
      border-radius: 999px;
      border: 3px solid var(--accent);
    }
    .right-grid {
      display: grid;
      gap: 16px;
    }
    .group h2 {
      margin: 0 0 12px;
      text-align: center;
      opacity: 0.45;
      letter-spacing: 0.06em;
      font-size: 1.15rem;
      font-weight: 500;
    }
    .button-row {
      display: flex;
      gap: 8px;
      justify-content: center;
      flex-wrap: wrap;
    }
    .button-row .btn {
      min-width: 128px;
    }
    .pad {
      width: 220px;
      margin: 0 auto;
      padding: 10px;
      border-radius: 28px;
      background: linear-gradient(rgba(40,41,46,1), rgba(40,41,46,0.3));
      box-shadow: 0 10px 10px rgba(0,0,0,0.15);
    }
    .pad-grid {
      display: grid;
      grid-template-columns: repeat(3, 70px);
      grid-template-rows: repeat(3, 70px);
    }
    .pad-btn {
      border: 0;
      background: transparent;
      color: var(--accent);
      cursor: pointer;
      font-weight: 800;
      font-size: 0.9rem;
      border-radius: 12px;
    }
    .pad-btn:hover { background: rgba(79,245,192,0.08); }
    .pad-btn.stop {
      background: rgba(79,245,192,0.1);
      border: 1px solid rgba(79,245,192,0.2);
    }
    .slider-grid {
      display: grid;
      gap: 14px;
      margin-top: 10px;
    }
    .slider-label {
      display: flex;
      justify-content: space-between;
      margin-bottom: 8px;
      font-weight: 700;
      color: var(--accent);
    }
    input[type=range] {
      width: 100%;
      accent-color: var(--accent);
    }
    .tiny-note {
      margin-top: 10px;
      font-size: 0.92rem;
      opacity: 0.65;
      text-align: center;
    }
    @media (max-width: 1100px) {
      .layout { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="card">
      <div class="hero">
        <div>
          <h1>Legacy Style</h1>
          <p>Trang này dựng lại bố cục và cảm giác của web legacy, nhưng toàn bộ điều khiển vẫn đi qua ROS stack mới. Đây là nền để build lại UI mới mà không phụ thuộc runtime cũ.</p>
        </div>
        <div class="links">
          <a href="/">New UI</a>
          <a href="/photo">Photos</a>
          <a href="/video">Videos</a>
          <a href="/settings">Settings</a>
          <a id="legacyFeedLink" href="#" target="_blank">Open Feed</a>
        </div>
      </div>
      <div class="layout">
        <div class="card">
          <div class="video-shell">
            <img id="legacyFeed" alt="CV Feed">
            <div class="overlay">
              <div class="overlay-top">
                <div class="stack">
                  <p>CPU: <span id="lgCpu">--</span></p>
                  <p>RAM: <span id="lgRam">--</span></p>
                </div>
                <div class="stack" style="text-align: right;">
                  <div class="status-pill"><span class="badge-dot"></span><span id="lgMode">ROS</span></div>
                  <p>RSSI: <span id="lgRssi">--</span></p>
                  <p>FPS: <span id="lgFps">--</span></p>
                  <p>TEMP: <span id="lgTemp">--</span></p>
                  <p>BAT: <span id="lgBattery">--</span></p>
                </div>
              </div>
              <div class="overlay-center">
                <div></div>
                <div class="focus-box"></div>
                <div></div>
              </div>
              <div class="overlay-bottom">
                <div class="stack">
                  <p>Zoom: <span>1x</span></p>
                  <p>Photos: <span id="lgPhotos">--</span></p>
                  <p>Videos: <span id="lgVideos">--</span></p>
                </div>
                <div class="stack" style="text-align: center;">
                  <p>Pan: <span id="lgPan">--</span></p>
                  <p>Tilt: <span id="lgTilt">--</span></p>
                </div>
                <div class="stack" style="text-align: right;">
                  <p>Target: <span id="lgTarget">--</span></p>
                  <p>Offset: <span id="lgOffset">--</span></p>
                </div>
              </div>
            </div>
          </div>
          <div class="video-actions">
            <button class="btn" id="legacyRecordBtn">Record</button>
            <button class="btn capture-btn" id="legacyCaptureBtn">Capture</button>
            <button class="btn" id="legacyRefreshBtn">Refresh</button>
          </div>
          <div class="tiny-note">Video feed lấy trực tiếp từ <code>cv_node</code> ở cổng <code>5051</code>.</div>
        </div>
        <div class="right-grid">
          <div class="card group">
            <h2>Drive Pad</h2>
            <div class="pad">
              <div class="pad-grid">
                <button class="pad-btn" data-linear="0.6" data-angular="0.35">LF</button>
                <button class="pad-btn" data-linear="0.8" data-angular="0">FW</button>
                <button class="pad-btn" data-linear="0.6" data-angular="-0.35">RF</button>
                <button class="pad-btn" data-linear="0" data-angular="0.55">LT</button>
                <button class="pad-btn stop" id="legacyStopBtn">STOP</button>
                <button class="pad-btn" data-linear="0" data-angular="-0.55">RT</button>
                <button class="pad-btn" data-linear="-0.6" data-angular="-0.35">LB</button>
                <button class="pad-btn" data-linear="-0.8" data-angular="0">BW</button>
                <button class="pad-btn" data-linear="-0.6" data-angular="0.35">RB</button>
              </div>
            </div>
            <div class="tiny-note">Pad này mô phỏng layout 9 hướng của giao diện cũ.</div>
          </div>

          <div class="card group">
            <h2>CV Mode</h2>
            <div class="button-row">
              <button class="btn legacy-cv-mode" data-mode="cv_none" data-lock="true" data-light="0">None</button>
              <button class="btn legacy-cv-mode" data-mode="cv_face" data-lock="false" data-light="1">Faces</button>
              <button class="btn legacy-cv-mode" data-mode="cv_clor" data-lock="false" data-light="1">Color</button>
            </div>
            <div class="button-row" style="margin-top: 8px;">
              <button class="btn legacy-cv-mode" data-mode="cv_auto" data-lock="false" data-light="0">Autodrive</button>
              <button class="btn legacy-cv-mode" data-mode="cv_objs" data-lock="true" data-light="0">Objects</button>
              <button class="btn legacy-cv-mode" data-mode="mp_hand" data-lock="false" data-light="0">Hand GS</button>
            </div>
            <div class="button-row" style="margin-top: 8px;">
              <button class="btn legacy-cv-mode" data-mode="mp_face" data-lock="true" data-light="0">MP Face</button>
              <button class="btn legacy-cv-mode" data-mode="mp_pose" data-lock="true" data-light="0">MP Pose</button>
            </div>
          </div>

          <div class="card group">
            <h2>Head Light Ctrl</h2>
            <div class="button-row">
              <button class="btn" id="legacyLightOffBtn">Off</button>
              <button class="btn" id="legacyLightAutoBtn">Auto</button>
              <button class="btn" id="legacyLightOnBtn">On</button>
            </div>
          </div>

          <div class="card group">
            <h2>PT Ctrl</h2>
            <div class="slider-grid">
              <div>
                <div class="slider-label"><span>Pan</span><span id="legacyPanOut">0</span></div>
                <input id="legacyPanRange" type="range" min="-180" max="180" step="1" value="0">
              </div>
              <div>
                <div class="slider-label"><span>Tilt</span><span id="legacyTiltOut">0</span></div>
                <input id="legacyTiltRange" type="range" min="-90" max="90" step="1" value="0">
              </div>
            </div>
            <div class="button-row" style="margin-top: 10px;">
              <button class="btn" id="legacyAheadBtn">Ahead</button>
              <button class="btn" id="legacySendPtBtn">Send PT</button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <script>
    const feedUrl = `http://${window.location.hostname}:5051/video_feed`;
    document.getElementById('legacyFeed').src = feedUrl;
    document.getElementById('legacyFeedLink').href = feedUrl;

    const panRange = document.getElementById('legacyPanRange');
    const tiltRange = document.getElementById('legacyTiltRange');
    const panOut = document.getElementById('legacyPanOut');
    const tiltOut = document.getElementById('legacyTiltOut');

    function updatePtLabels() {
      panOut.textContent = panRange.value;
      tiltOut.textContent = tiltRange.value;
    }

    panRange.addEventListener('input', updatePtLabels);
    tiltRange.addEventListener('input', updatePtLabels);
    updatePtLabels();

    async function postJson(path, payload) {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      return response.json();
    }

    async function getJson(path) {
      const response = await fetch(path);
      return response.json();
    }

    async function refreshLegacy() {
      try {
        const [state, cv, target, photos, videos] = await Promise.all([
          getJson('/state'),
          getJson('/api/cv/status'),
          getJson('/api/cv/tracking_target'),
          getJson('/api/photos'),
          getJson('/api/videos')
        ]);
        document.getElementById('lgBattery').textContent = `${Number(state.battery_voltage).toFixed(2)} V`;
        document.getElementById('lgPan').textContent = Number(state.gimbal_pan).toFixed(1);
        document.getElementById('lgTilt').textContent = Number(state.gimbal_tilt).toFixed(1);
        document.getElementById('lgMode').textContent = cv.mode;
        document.getElementById('lgFps').textContent = Number(cv.fps).toFixed(1);
        document.getElementById('lgTarget').textContent = cv.target_present ? cv.target_kind : 'none';
        document.getElementById('lgOffset').textContent = cv.target_present
          ? `${Number(cv.target_offset_x).toFixed(1)}, ${Number(cv.target_offset_y).toFixed(1)}`
          : '--';
        document.getElementById('lgPhotos').textContent = String(photos.length);
        document.getElementById('lgVideos').textContent = String(videos.length);
        document.getElementById('lgCpu').textContent = '--';
        document.getElementById('lgRam').textContent = '--';
        document.getElementById('lgRssi').textContent = '--';
        document.getElementById('lgTemp').textContent = '--';
        if (target && target.target_present) {
          document.getElementById('lgTarget').textContent = `${target.kind}`;
        }
      } catch (error) {}
    }

    async function sendMotion(linear, angular, source = 'legacy_style_ui') {
      await postJson('/api/motion', {
        source,
        linear,
        angular,
        priority: 1,
        mode: 'manual',
        timeout_ms: 400,
        frame_id: 'base_link'
      });
    }

    async function sendStop() {
      await postJson('/api/stop', { active: true });
      setTimeout(() => postJson('/api/stop', { active: false }), 120);
    }

    async function sendGimbal(pan, tilt, source = 'legacy_style_ui') {
      await postJson('/api/gimbal', {
        source,
        pan,
        tilt,
        speed: 18,
        accel: 0,
        mode: 'manual',
        timeout_ms: 1000
      });
    }

    async function setCvMode(mode, motionLock, lightMode, targetColor = null) {
      const payload = { mode, motion_lock: motionLock, light_mode: lightMode };
      if (targetColor) payload.target_color = targetColor;
      await postJson('/api/cv/mode', payload);
      await refreshLegacy();
    }

    document.querySelectorAll('.pad-btn[data-linear]').forEach((button) => {
      button.addEventListener('click', async () => {
        await sendMotion(Number(button.dataset.linear), Number(button.dataset.angular));
      });
    });
    document.getElementById('legacyStopBtn').addEventListener('click', sendStop);
    document.getElementById('legacyAheadBtn').addEventListener('click', async () => {
      panRange.value = '0';
      tiltRange.value = '0';
      updatePtLabels();
      await sendGimbal(0, 0, 'legacy_style_ahead');
    });
    document.getElementById('legacySendPtBtn').addEventListener('click', async () => {
      await sendGimbal(Number(panRange.value), Number(tiltRange.value));
    });
    document.getElementById('legacyRefreshBtn').addEventListener('click', refreshLegacy);
    document.getElementById('legacyCaptureBtn').addEventListener('click', async () => {
      await postJson('/api/cv/test_intent', { kind: 'lights', source: 'legacy_capture_flash', base_pwm: 255, head_pwm: 255, timeout_ms: 120 });
    });
    document.getElementById('legacyRecordBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', true, 2);
    });
    document.getElementById('legacyLightOffBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', true, 0);
    });
    document.getElementById('legacyLightAutoBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', true, 1);
    });
    document.getElementById('legacyLightOnBtn').addEventListener('click', async () => {
      await setCvMode('cv_none', true, 2);
    });
    document.querySelectorAll('.legacy-cv-mode').forEach((button) => {
      button.addEventListener('click', async () => {
        await setCvMode(
          button.dataset.mode,
          button.dataset.lock === 'true',
          Number(button.dataset.light)
        );
      });
    });

    refreshLegacy();
    setInterval(refreshLegacy, 1000);
  </script>
</body>
</html>
"""

    def render_video_page(self):
        return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rasprover Videos</title>
  <style>
    body { font-family: "Trebuchet MS", sans-serif; margin: 0; background: #f7f3ea; color: #1c2b20; }
    .wrap { max-width: 980px; margin: 0 auto; padding: 24px; }
    .top { display: flex; gap: 12px; flex-wrap: wrap; align-items: center; justify-content: space-between; }
    .btn { display: inline-block; padding: 10px 14px; border-radius: 14px; background: #355647; color: #fff; text-decoration: none; border: 0; cursor: pointer; }
    .list { display: grid; gap: 14px; margin-top: 20px; }
    .row { background: #fff; border-radius: 18px; padding: 14px; box-shadow: 0 10px 24px rgba(0,0,0,0.08); display: flex; justify-content: space-between; gap: 12px; align-items: center; }
    .name { font-weight: 700; word-break: break-all; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <h1>Video Files</h1>
      <div>
        <a class="btn" href="/">Home</a>
        <a class="btn" href="/photo">Photos</a>
      </div>
    </div>
    <div id="count"></div>
    <div class="list" id="list"></div>
  </div>
  <script>
    async function loadVideos() {
      const videos = await fetch('/api/videos').then(r => r.json());
      document.getElementById('count').textContent = `Total: ${videos.length}`;
      const list = document.getElementById('list');
      list.innerHTML = '';
      for (const name of videos) {
        const row = document.createElement('div');
        row.className = 'row';
        row.innerHTML = `
          <div class="name">${name}</div>
          <div>
            <a class="btn" href="/videos/${encodeURIComponent(name)}" target="_blank">Open</a>
            <button class="btn">Delete</button>
          </div>
        `;
        row.querySelector('button').addEventListener('click', async () => {
          await fetch('/api/videos/delete', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ filename: name }) });
          loadVideos();
        });
        list.appendChild(row);
      }
    }
    loadVideos();
  </script>
</body>
</html>
"""

    def render_settings_page(self):
        return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rasprover Servo Setup</title>
  <style>
    body { font-family: "Trebuchet MS", sans-serif; margin: 0; background: #f7f3ea; color: #1c2b20; }
    .wrap { max-width: 920px; margin: 0 auto; padding: 24px; }
    .btn { display: inline-block; padding: 10px 14px; border-radius: 14px; background: #355647; color: #fff; text-decoration: none; border: 0; cursor: pointer; margin-right: 8px; }
    .step { background: #fff; padding: 18px; border-radius: 18px; box-shadow: 0 10px 24px rgba(0,0,0,0.08); margin-top: 14px; }
    .step h2 { margin-top: 0; }
    #status { margin-top: 16px; font-weight: 700; }
  </style>
</head>
<body>
  <div class="wrap">
    <a class="btn" href="/">Home</a>
    <h1>Bus Servo Initialization</h1>
    <div class="step"><h2>1. Set Pan ID</h2><p>Disconnect Tilt Servo first, then set the current servo to pan ID 2.</p><button class="btn" onclick="sendAction('set_pan_id')">Set Pan ID</button></div>
    <div class="step"><h2>2. Release Torque</h2><p>Unlock both servos so you can adjust them by hand.</p><button class="btn" onclick="sendAction('release')">Release</button></div>
    <div class="step"><h2>3. Middle Set</h2><p>After aiming the camera straight ahead, save the current position as middle.</p><button class="btn" onclick="sendAction('middle_set')">Middle Set</button></div>
    <div class="step"><h2>4. Restore Tilt ID</h2><p>Only use this if both servos ended up with ID 2 and you need to restore the tilt servo to ID 1.</p><button class="btn" onclick="sendAction('set_tilt_id')">Set Tilt ID</button></div>
    <div id="status">Ready</div>
  </div>
  <script>
    async function sendAction(action) {
      const response = await fetch('/api/settings/servo', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ source: 'web_settings', action, servo_id: 255, old_id: 255, status: 0 })
      });
      const data = await response.json();
      document.getElementById('status').textContent = data.ok ? `Sent: ${action}` : `Failed: ${action}`;
    }
  </script>
</body>
</html>
"""

    def destroy_node(self):
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
