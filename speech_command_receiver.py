import socket
import pyttsx3

# Setup
UDP_IP = "0.0.0.0"     # Listen on all interfaces
UDP_PORT = 5006        # Must match ROS sender node's target_port

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Initialize TTS engine once (reuse for efficiency)
engine = pyttsx3.init()
engine.setProperty('rate', 175)  # Adjust speaking rate if you want
engine.setProperty('volume', 1.0)

def speak(text):
    engine.say(text)
    engine.runAndWait()

def main():
    print(f"Listening for UDP messages on port {UDP_PORT}...\n")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode().strip()
            print(f"[RECEIVED FROM ROS] {message}")
            speak(message)
        except KeyboardInterrupt:
            print("\nExiting ...")
            break
        except Exception as e:
            print(f"******Error: {e}")

if __name__ == "__main__":
    main()
