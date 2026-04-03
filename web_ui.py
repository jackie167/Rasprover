import asyncio
import json
import os
import uuid

import audio_ctrl
from aiortc import RTCPeerConnection, RTCSessionDescription
from flask import Flask, Response, jsonify, render_template, request, send_from_directory
from flask_socketio import SocketIO
from werkzeug.utils import secure_filename


class WebUI:
    def __init__(self, project_path, config_path, app_config, cmd_mux, cvf, system_info, state_store):
        self.project_path = project_path
        self.config_path = config_path
        self.app_config = app_config
        self.cmd_mux = cmd_mux
        self.cvf = cvf
        self.system_info = system_info
        self.state_store = state_store

        self.upload_folder = os.path.join(project_path, 'sounds', 'others')
        self.photo_dir = os.path.join(project_path, 'templates', 'pictures')
        self.video_dir = os.path.join(project_path, 'templates', 'videos')

        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.active_pcs = {}
        self.max_connections = 1

        self._register_routes()
        self._register_socket_handlers()

    def _register_routes(self):
        @self.app.route('/')
        def index():
            audio_ctrl.play_random_audio("connected", False)
            return render_template('index.html')

        @self.app.route('/config')
        def get_config():
            with open(self.config_path, 'r', encoding='utf-8') as file:
                return file.read()

        @self.app.route('/<path:filename>')
        def serve_static(filename):
            return send_from_directory('templates', filename)

        @self.app.route('/get_photo_names')
        def get_photo_names():
            photo_files = sorted(
                os.listdir(self.photo_dir),
                key=lambda name: os.path.getmtime(os.path.join(self.photo_dir, name)),
                reverse=True,
            )
            return jsonify(photo_files)

        @self.app.route('/delete_photo', methods=['POST'])
        def delete_photo():
            filename = request.form.get('filename')
            try:
                os.remove(os.path.join(self.photo_dir, filename))
                return jsonify(success=True)
            except Exception as e:
                print(e)
                return jsonify(success=False)

        @self.app.route('/videos/<path:filename>')
        def videos(filename):
            return send_from_directory(self.video_dir, filename)

        @self.app.route('/get_video_names')
        def get_video_names():
            video_files = sorted(
                [filename for filename in os.listdir(self.video_dir) if filename.endswith('.mp4')],
                key=lambda filename: os.path.getctime(os.path.join(self.video_dir, filename)),
                reverse=True,
            )
            return jsonify(video_files)

        @self.app.route('/delete_video', methods=['POST'])
        def delete_video():
            filename = request.form.get('filename')
            try:
                os.remove(os.path.join(self.video_dir, filename))
                return jsonify(success=True)
            except Exception as e:
                print(e)
                return jsonify(success=False)

        @self.app.route('/offer', methods=['POST'])
        def offer_route():
            return self.offer()

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/send_command', methods=['POST'])
        def handle_command():
            command = request.form['command']
            print("Received command:", command)
            try:
                self.cmd_mux.cmdline_ctrl(command, source="web_command")
            except Exception as e:
                print(f"[web_ui.handle_command] error: {e}")
            return jsonify({"status": "success", "message": "Command received"})

        @self.app.route('/getAudioFiles', methods=['GET'])
        def get_audio_files():
            files = [
                f for f in os.listdir(self.upload_folder)
                if os.path.isfile(os.path.join(self.upload_folder, f)) and f.endswith(('.mp3', '.wav'))
            ]
            return jsonify(files)

        @self.app.route('/uploadAudio', methods=['POST'])
        def upload_audio():
            if 'file' not in request.files:
                return jsonify({'error': 'No file part'})
            file = request.files['file']
            if file.filename == '':
                return jsonify({'error': 'No selected file'})
            filename = secure_filename(file.filename)
            file.save(os.path.join(self.upload_folder, filename))
            return jsonify({'success': 'File uploaded successfully'})

        @self.app.route('/playAudio', methods=['POST'])
        def play_audio():
            audio_file = request.form['audio_file']
            file_path = os.path.join(self.upload_folder, audio_file)
            self.cmd_mux.play_audio_file(file_path)
            return jsonify({'success': 'Audio is playing'})

        @self.app.route('/stop_audio', methods=['POST'])
        def audio_stop():
            self.cmd_mux.stop_audio()
            return jsonify({'success': 'Audio stop'})

        @self.app.route('/settings/<path:filename>')
        def serve_static_settings(filename):
            return send_from_directory('templates', filename)

    def _register_socket_handlers(self):
        @self.socketio.on('json', namespace='/json')
        def handle_socket_json(payload):
            try:
                self.cmd_mux.handle_json_command(payload, source="web_json")
            except Exception as e:
                print("Error handling JSON data:", e)

        @self.socketio.on('message', namespace='/ctrl')
        def handle_socket_cmd(message):
            try:
                json_data = json.loads(message)
            except json.JSONDecodeError:
                print("Error decoding JSON.[web_ui.handle_socket_cmd]")
                return

            cmd_a = float(json_data.get("A", 0))
            if self.cmd_mux.handle_action(cmd_a, source="web_socket"):
                self.update_data_websocket_single()

    def generate_frames(self):
        while True:
            frame = self.cvf.frame_process()
            try:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                print("An [web_ui.generate_frames] error occurred:", e)

    def manage_connections(self, pc_id, pc):
        if len(self.active_pcs) >= self.max_connections:
            oldest_pc_id = next(iter(self.active_pcs))
            old_pc = self.active_pcs.pop(oldest_pc_id)
            old_pc.close()
        self.active_pcs[pc_id] = pc

    async def offer_async(self):
        params = request.get_json(force=True)
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc = RTCPeerConnection()
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        pc_id = pc_id[:8]
        self.manage_connections(pc_id, pc)
        await pc.createOffer(offer)
        await pc.setLocalDescription(offer)
        response_data = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        return jsonify(response_data)

    def offer(self):
        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)
            return loop.run_until_complete(self.offer_async())
        finally:
            loop.close()

    def update_data_websocket_single(self):
        try:
            self.cmd_mux.sync_cv_state()
            self.state_store.update_system_info(self.system_info)
            state = self.state_store.snapshot()
            system_state = state['system']
            cv_state = state['cv']
            robot_state = state['robot']
            socket_data = {
                self.app_config.fb('picture_size'): system_state['picture_size'],
                self.app_config.fb('video_size'): system_state['video_size'],
                self.app_config.fb('cpu_load'): system_state['cpu_load'],
                self.app_config.fb('cpu_temp'): system_state['cpu_temp'],
                self.app_config.fb('ram_usage'): system_state['ram_usage'],
                self.app_config.fb('wifi_rssi'): system_state['wifi_rssi'],
                self.app_config.fb('led_mode'): cv_state['led_mode'],
                self.app_config.fb('detect_type'): cv_state['mode'],
                self.app_config.fb('detect_react'): cv_state['detection_reaction_mode'],
                self.app_config.fb('pan_angle'): cv_state['pan_angle'],
                self.app_config.fb('tilt_angle'): cv_state['tilt_angle'],
                self.app_config.fb('base_voltage'): robot_state['base_voltage'],
                self.app_config.fb('video_fps'): cv_state['video_fps'],
                self.app_config.fb('cv_movtion_mode'): cv_state['motion_lock'],
                self.app_config.fb('base_light'): robot_state['base_light_status'],
            }
            self.socketio.emit('update', socket_data, namespace='/ctrl')
        except Exception as e:
            print("An [web_ui.update_data_websocket_single] error occurred:", e)
