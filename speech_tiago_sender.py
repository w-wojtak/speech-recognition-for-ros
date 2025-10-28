import socket
import json
import pyaudio
from vosk import Model, KaldiRecognizer
import pyttsx3

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
    "next_object": "Next object.",
    "give_motor": "Here is the motor.",
    "give_load": "Here is the load.",
    "give_bearing": "Here is the bearing.",
    "give_base": "Here is the base."
}

# Grammar: only recognize these phrases
GRAMMAR = '''
[
    "next object", "next", "object",
    "give me the motor", "give me the load", "give me the bearing", "give me the base",
    "I need the motor", "I need  the load", "I need  the bearing", "I need  the base",
    "motor", "load", "lo", "low", "bearing", "bear", "base", "bass"
]
'''

tts_engine = pyttsx3.init()
tts_engine.setProperty('rate', 180)

def find_command(text):
    text = text.lower().strip()
    if text in COMMAND_MAP:
        return COMMAND_MAP[text]
    for key in COMMAND_MAP.keys():
        if key in text:
            return COMMAND_MAP[key]
    return None

def speak(text):
    tts_engine.say(text)
    tts_engine.runAndWait()

def main():
    MODEL_PATH = "vosk-model-small-en-us-0.15"
    print(f"Loading Vosk model from {MODEL_PATH}...")
    
    try:
        model = Model(MODEL_PATH)
    except Exception as e:
        print(f"Failed to load model: {e}")
        return
    
    recognizer = KaldiRecognizer(model, 16000)
    recognizer.SetGrammar(GRAMMAR)  # ⚡ Constrain to your commands only!
    
    mic = pyaudio.PyAudio()
    stream = mic.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=4096
    )
    stream.start_stream()
    
    print("🎤 Speech recognition ready! Say a command:")
    
    try:
        while True:
            data = stream.read(4096, exception_on_overflow=False)
            
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "")
                
                if text:
                    print(f"✓ Recognized: {text}")
                    cmd = find_command(text)
                    
                    if cmd:
                        print(f"→ Executing: '{cmd}'")
                        sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
                        speak(RESPONSES.get(cmd, ""))
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        stream.stop_stream()
        stream.close()
        mic.terminate()

if __name__ == "__main__":
    main()