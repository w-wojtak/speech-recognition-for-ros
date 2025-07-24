import speech_recognition as sr

# Mapping recognized variants to normalized commands
COMMAND_MAP = {
    "let's start": "start",
    "start": "start",
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

def find_command(text):
    text = text.lower()
    # Try exact match first
    if text in COMMAND_MAP:
        return COMMAND_MAP[text]
    # Try partial match (key contained in recognized text)
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

def main():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("Say a command:")

    while True:
        spoken_text = recognize_speech_from_mic(recognizer, mic)
        if spoken_text:
            cmd = find_command(spoken_text)
            if cmd:
                print(f"Command recognized and normalized: '{cmd}'")
                # TODO: send `cmd` string to ROS1/ROS2 here
            else:
                print("Command not recognized as a valid command.")

if __name__ == "__main__":
    main()
