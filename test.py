from vosk import Model, KaldiRecognizer
import wave
import json

# Load the Vosk model
model_path = "vosk-model-small-en-us-0.15"
print("Loading model...")
model = Model(model_path)
print("Model loaded successfully!")

# Test with a short WAV file (16kHz mono)
wf = wave.open("test.wav", "rb")

# Ensure correct audio format
if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getframerate() not in [8000, 16000, 44100]:
    raise ValueError("Audio file must be WAV format mono PCM (16-bit)")

rec = KaldiRecognizer(model, wf.getframerate())

print("Recognizing speech...")
while True:
    data = wf.readframes(4000)
    if len(data) == 0:
        break
    if rec.AcceptWaveform(data):
        print(json.loads(rec.Result()))

# Final result
print(json.loads(rec.FinalResult()))
