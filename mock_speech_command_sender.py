import socket
import keyboard  # pip install keyboard

UDP_IP = "172.19.251.174"  # Your WSL eth0 IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Map keys to commands
KEY_COMMAND_MAP = {
    's': "start",
    'f': "finished",
    'n': "next_object",
    'm': "give_motor",
    'l': "give_load",
    'b': "give_bearing",
    'a': "give_base",  # 'a' as in base (since 'b' is bearing)
    'q': "quit"
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

def main():
    print("Mock command sender is running.")
    print("Press a key to send a command:")
    print("s: start | f: finished | n: next object | m: motor | l: load | b: bearing | a: base | q: quit")

    while True:
        event = keyboard.read_event()
        if event.event_type == keyboard.KEY_DOWN:
            key = event.name.lower()
            if key in KEY_COMMAND_MAP:
                cmd = KEY_COMMAND_MAP[key]
                if cmd == "quit":
                    print("Exiting.")
                    break
                print(f"[INFO] Sending command: '{cmd}'")
                sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
                response = RESPONSES.get(cmd, f"Command '{cmd}' sent.")
                print(f"[ROBOT] {response}")

if __name__ == "__main__":
    main()
