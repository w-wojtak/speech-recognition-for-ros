import speech_recognition as sr

# Define your commands here
COMMANDS = [
    "let's start",
    "i have finished",
    "next object",
    "give me the motor",
    "give me the load",
    "give me the bearing",
    "give me the base"
]


def recognize_speech_from_mic(recognizer, microphone):
    """Capture speech from mic and return recognized text or None."""
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

def main():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("Say a command:")

    while True:
        command = recognize_speech_from_mic(recognizer, mic)
        if command:
            # Check if recognized command matches any in the list (simple matching)
            for cmd in COMMANDS:
                if cmd in command:
                    print(f"Command recognized: '{cmd}'")
                    # TODO: send cmd to ROS system via network here
                    break
            else:
                print("Command not recognized as a valid command.")

if __name__ == "__main__":
    main()
