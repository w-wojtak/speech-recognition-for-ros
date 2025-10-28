import socket
import speech_recognition as sr  # pip install speechrecognition
import pyttsx3 # pip install pyttsx3
# pip install pyaudio

UDP_IP = "172.19.251.174"  # Your WSL eth0 IP (from `ip addr show eth0`)
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

COMMAND_MAP = {
    "let's start": "start",
    "start": "start",
    "let's go": "start",
    "let's begin": "start",
    "begin": "start",
    "i have finished": "finished",
    "have finished": "finished",
    "finished": "finished",
    "next object": "next_object",
    "next": "next_object",
    "object": "next_object",
    "give me the motor": "give_motor",
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
    "start": "Starting.",
    "finished": "Finished.",
    "next_object": "Next object.",
    "give_motor": "Here is the motor.",
    "give_load": "Here is the load.",
    "give_bearing": "Here is the bearing.",
    "give_base": "Here is the base."
}

def find_command(text):
    text = text.lower()
    if text in COMMAND_MAP:
        return COMMAND_MAP[text]
    for key in COMMAND_MAP.keys():
        if key in text:
            return COMMAND_MAP[key]
    return None

def recognize_speech_from_mic(recognizer, microphone):
    with microphone as source:
        print("Listening...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)
    try:
        text = recognizer.recognize_google(audio).lower()
        print(f"Recognized: {text}")
        return text
    except sr.UnknownValueError:
        print("Sorry, I did not understand that.")
    except sr.RequestError as e:
        print(f"Could not request results; {e}")
    return None

def speak(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()

def main():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    print("Speech recognition is running. Say a command:")

    while True:
        spoken_text = recognize_speech_from_mic(recognizer, mic)
        if not spoken_text:
            speak("I didn't catch that.")
            continue
        cmd = find_command(spoken_text)
        if cmd:
            print(f"[INFO] Command recognized: '{cmd}'")
            sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))  # send UDP to ROS2 WSL IP
            response = RESPONSES.get(cmd, f"Command {cmd} sent.")
            speak(response)
        else:
            print("[INFO] Command not recognized.")
            speak("Command not recognized.")

if __name__ == "__main__":
    main()
