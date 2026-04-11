import os
import random
import sys
import threading
import time
from pathlib import Path

import pygame
import pyttsx3
import yaml

FALLBACK_ROOT = Path(__file__).resolve().parents[4]
if str(FALLBACK_ROOT) not in sys.path:
    sys.path.insert(0, str(FALLBACK_ROOT))

from repo_paths import find_repo_root

REPO_ROOT = find_repo_root(start_path=__file__, fallback_root=FALLBACK_ROOT)
usb_connected = False

with open(REPO_ROOT / 'config.yaml', 'r', encoding='utf-8') as yaml_file:
    config = yaml.safe_load(yaml_file)

current_path = str(REPO_ROOT)

try:
    pygame.mixer.init()
    pygame.mixer.music.set_volume(config['audio_config']['default_volume'])
    usb_connected = True
    print('audio usb connected')
except Exception:
    usb_connected = False
    print('audio usb not connected')

play_audio_event = threading.Event()
min_time_bewteen_play = config['audio_config']['min_time_bewteen_play']

engine = pyttsx3.init()
engine.setProperty('rate', config['audio_config']['speed_rate'])


def play_audio(input_audio_file):
    if not usb_connected:
        return
    try:
        pygame.mixer.music.load(input_audio_file)
        pygame.mixer.music.play()
    except Exception:
        play_audio_event.clear()
        return
    while pygame.mixer.music.get_busy():
        pass
    time.sleep(min_time_bewteen_play)
    play_audio_event.clear()


def play_random_audio(input_dirname, force_flag):
    if not usb_connected:
        return
    if play_audio_event.is_set() and not force_flag:
        return
    audio_dir = os.path.join(current_path, "sounds", input_dirname)
    audio_files = [f for f in os.listdir(audio_dir) if f.endswith((".mp3", ".wav"))]
    audio_file = random.choice(audio_files)
    play_audio_event.set()
    audio_thread = threading.Thread(target=play_audio, args=(os.path.join(audio_dir, audio_file),))
    audio_thread.start()


def play_audio_thread(input_file):
    if not usb_connected:
        return
    if play_audio_event.is_set():
        return
    play_audio_event.set()
    audio_thread = threading.Thread(target=play_audio, args=(input_file,))
    audio_thread.start()


def play_file(audio_file):
    if not usb_connected:
        return
    audio_file = os.path.join(current_path, "sounds", audio_file)
    play_audio_thread(audio_file)


def get_mixer_status():
    if not usb_connected:
        return
    return pygame.mixer.music.get_busy()


def set_audio_volume(input_volume):
    if not usb_connected:
        return
    input_volume = float(input_volume)
    if input_volume > 1:
        input_volume = 1
    elif input_volume < 0:
        input_volume = 0
    pygame.mixer.music.set_volume(input_volume)


def set_min_time_between(input_time):
    if not usb_connected:
        return
    global min_time_bewteen_play
    min_time_bewteen_play = input_time


def play_speech(input_text):
    if not usb_connected:
        return
    engine.say(input_text)
    engine.runAndWait()
    play_audio_event.clear()


def play_speech_thread(input_text):
    if not usb_connected:
        return
    if play_audio_event.is_set():
        return
    play_audio_event.set()
    speech_thread = threading.Thread(target=play_speech, args=(input_text,))
    speech_thread.start()


def stop():
    if not usb_connected:
        return
    pygame.mixer.music.stop()
    play_audio_event.clear()
