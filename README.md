# Speech Command Module for ROS

This Python-based speech recognition module captures spoken commands on a Windows machine, recognizes specific phrases using the Google Web Speech API, and sends the corresponding command to a ROS1 or ROS2 system over UDP.

## Features

- Uses the Google Web Speech API via `speech_recognition` library
- Recognizes a set of predefined commands (with variants)
- Normalizes and maps spoken phrases to compact robot-friendly keywords
- Sends commands via UDP to a listener node in either ROS1 or ROS2
- Supports text-to-speech feedback using `pyttsx3`

## Example Commands

- "Let's start" → `start`
- "I have finished" → `finished`
- "Next object" → `next_object`
- "Give me the motor" → `give_motor`

## Architecture

[Windows PC]

Speech Recognition (Python)

↓ UDP

[Ubuntu w/ ROS1 or ROS2]

Listener node processes command

## How to Use

1. Install dependencies:

```
pip install speechrecognition pyttsx3
```

Run the script on Windows:

```
python speech_command_sender.py
```

Start the listener node in ROS1/ROS2 on Ubuntu:

```
rosrun your_package ros1_udp_listener_node  # for ROS1
ros2 run your_package udp_listener_node     # for ROS2
```

Speak a supported command!