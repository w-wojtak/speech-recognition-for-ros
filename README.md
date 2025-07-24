# Speech Command Module for ROS

This Python-based speech recognition module captures spoken commands on a Windows machine, recognizes specific phrases using the Google Web Speech API, and sends the corresponding command to a ROS1 or ROS2 system over UDP.

## Features

- Speech recognition using the **Google Web Speech API**
- Supports **command variants** and fuzzy matching
- Voice feedback using **pyttsx3**
- Sends compact commands via **UDP** to ROS nodes
- **Receives messages from ROS** (and speaks them aloud)
- Works with **ROS1 and ROS2**, on the same or different machines

## Example Commands

- "Let's start" → `start`
- "I have finished" → `finished`
- "Next object" → `next_object`
- "Give me the motor" → `give_motor`

## Architecture

#### Windows PC

Speech Command Sender (Python)

Sends voice command over UDP

#### Ubuntu with ROS1 or ROS2

UDP Listener Node receives command

Publishes to a ROS topic

#### Windows PC

Speech Command Receiver (Python)

Receives message over UDP and plays TTS feedback

## How to Use

1. Install dependencies:

```
pip install speechrecognition pyttsx3
```

Run the Speech Sender (on Windows):

```
python speech_command_sender.py
```

Start the listener node in ROS1/ROS2 on Ubuntu:

```
rosrun your_package ros1_udp_listener_node  # for ROS1
ros2 run your_package udp_listener_node     # for ROS2
```

Speak a supported command!

Run Receiver to Speak ROS Messages (on Windows)

```
python speech_command_receiver.py
```

Make sure ROS is sending messages via UDP to the IP and port defined in `speech_command_receiver.py`.


## Notes
Make sure your UDP IP and ports are correct for your network setup.

Use `ip addr show eth0` inside WSL to find your Windows IP from Ubuntu side.

