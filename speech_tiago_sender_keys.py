import socket
import json
import pyaudio
from vosk import Model, KaldiRecognizer
import pyttsx3
import threading
import queue
import keyboard

UDP_IP = "172.19.251.174"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

COMMAND_MAP = {
    "next object": "next_object",
    "next": "next_object",
    "object": "next_object",
    "give me the motor": "give_motor",
    "I need the motor": "give_motor",
    "I need the load": "give_load",
    "I need the bearing": "give_bearing",
    "I need the base": "give_base",
    "I take the motor": "give_motor",
    "I take the load": "give_load",
    "I take the bearing": "give_bearing",
    "I take the base": "give_base",
    "give me the load": "give_load",
    "give me the bearing": "give_bearing",
    "give me the base": "give_base",
    "motor": "give_motor",
    "load": "give_load",
    "lo": "give_load",
    "low": "give_load",
    "bearing": "give_bearing",
    "bear": "give_bearing",
    "base": "give_base",
    "bass": "give_base"
}

RESPONSES = {
    "give_motor": "Here is the motor.",
    "give_load": "Here is the load.",
    "give_bearing": "Here is the bearing.",
    "give_base": "Here is the base."
}

GRAMMAR = '''
[
    "next object", "next", "object",
    "give me the motor", "give me the load", "give me the bearing", "give me the base",
    "I need the motor", "I need  the load", "I need  the bearing", "I need the base",
    "I take the motor", "I take  the load", "I take  the bearing", "I take the base",
    "motor", "load", "lo", "low", "bearing", "bear", "base", "bass"
]
'''

# -------------------- TTS with a single thread --------------------
tts_engine = pyttsx3.init()
tts_engine.setProperty('rate', 180)
speech_queue = queue.Queue()
speech_enabled = True   # speech toggle variable

def tts_worker():
    while True:
        text = speech_queue.get()
        if text is None:
            break
        tts_engine.say(text)
        tts_engine.runAndWait()
        speech_queue.task_done()

threading.Thread(target=tts_worker, daemon=True).start()

def speak(text):
    if text:
        speech_queue.put(text)

def send_command(cmd):
    print(f"----> Sending: '{cmd}'")
    sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    speak(RESPONSES.get(cmd, ""))

def keyboard_listener():
    global speech_enabled
    print("Keyboard mode active. Press keys:")
    print("  M = motor, L = load, B = bearing, A = base, N = next object")
    print("  S = toggle speech on/off, Q = quit")

    pressed = set()

    while True:
        event = keyboard.read_event(suppress=True)
        if event.event_type == keyboard.KEY_DOWN:
            key = event.name.lower()
            if key not in pressed:
                pressed.add(key)
                if key == 'm':
                    send_command("give_motor")
                elif key == 'l':
                    send_command("give_load")
                elif key == 'b':
                    send_command("give_bearing")
                elif key == 'a':
                    send_command("give_base")
                elif key == 'n':
                    send_command("next_object")
                elif key == 's':
                    speech_enabled = not speech_enabled
                    status = "ON " if speech_enabled else "OFF "
                    print(f"\nSpeech listening toggled {status}\n")
                elif key == 'q':
                    print("Exiting keyboard mode.")
                    break
        elif event.event_type == keyboard.KEY_UP:
            pressed.discard(event.name.lower())

def main():
    global speech_enabled
    MODEL_PATH = "vosk-model-small-en-us-0.15"
    print(f"Loading Vosk model from {MODEL_PATH}...")

    try:
        model = Model(MODEL_PATH)
    except Exception as e:
        print(f"Failed to load model: {e}")
        return

    recognizer = KaldiRecognizer(model, 16000)
    recognizer.SetGrammar(GRAMMAR)

    mic = pyaudio.PyAudio()
    stream = mic.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=4096
    )
    stream.start_stream()

    threading.Thread(target=keyboard_listener, daemon=True).start()
    print("Speech recognition ready! Say a command or use keyboard shortcuts...")

    try:
        while True:
            if not speech_enabled:
                continue  # skip recognition entirely

            data = stream.read(4096, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "")
                if text:
                    print(f"*** Recognized: {text}")
                    for phrase, cmd in COMMAND_MAP.items():
                        if phrase in text.lower():
                            send_command(cmd)
                            break
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stream.stop_stream()
        stream.close()
        mic.terminate()
        speech_queue.put(None)

if __name__ == "__main__":
    main()