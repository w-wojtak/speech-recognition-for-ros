import socket
import pyttsx3

# Setup
UDP_IP = "0.0.0.0"      # Listen on all interfaces
UDP_PORT = 5006         # Port used by ROS to send messages (choose different from sender's)

# Create socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def speak(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()

def main():
    print(f"UDP receiver listening on port {UDP_PORT}...")
    while True:
        data, addr = sock.recvfrom(1024)
        message = data.decode()
        print(f"[RECEIVED FROM ROS] {message}")
        speak(message)

if __name__ == "__main__":
    main()
