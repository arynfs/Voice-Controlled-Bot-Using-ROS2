# ROS2 Voice Command Bot

Control a TurtleBot3 robot in Gazebo using your voice.

## Requirements

- ROS2 Humble
- Python 3.10
- Vosk Model (download and extract into `models/`):
  [Vosk Model](https://alphacephei.com/vosk/models)

## Setup

1. Clone the repository into your ROS2 workspace:

    ```bash
    cd ~/voice_ws/src
    git clone https://github.com/yourusername/ros2-voice-cmd-bot.git
    ```

2. Build the workspace:

    ```bash
    cd ~/voice_ws
    colcon build
    ```

3. Source the setup file:

    ```bash
    source install/setup.bash
    ```

4. Download the Vosk Model (e.g., `vosk-model-small-en-us-0.15`) and extract it into the `models/` directory:

    ```bash
    mkdir -p ~/voice_ws/src/voice_cmd_bot/models
    cd ~/voice_ws/src/voice_cmd_bot/models
    # Download and extract model (use the appropriate link)
    wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
    unzip vosk-model-small-en-us-0.15.zip
    ```

5. The model should now be in the `~/voice_ws/src/voice_cmd_bot/models/vosk-model-small-en-us-0.15` folder.

## Run

To simulate the bot using voice commands, run the following in separate terminals:

1. **Terminal 1** – Launch the Gazebo simulation:

    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. **Terminal 2** – Start the voice command recognition node:

    ```bash
    ros2 run voice_cmd_bot voice_node
    ```

3. **Terminal 3** – Start the command mapper to convert voice commands to robot motion:

    ```bash
    ros2 run voice_cmd_bot command_mapper
    ```

## Voice Commands

- Say **"forward"** to move the robot forward.
- Say **"backward"** to move the robot backward.
- Say **"left"** to turn the robot left.
- Say **"right"** to turn the robot right.
- Say **"stop"** to stop the robot.

The bot should respond to these voice commands by moving accordingly in the Gazebo simulation.

## Troubleshooting

- Ensure the Vosk model is downloaded and extracted correctly.
- Verify your microphone is working and properly configured in VirtualBox (enable audio input).
- If you encounter issues with dependencies, check that all necessary packages (`sounddevice`, `vosk`, `rclpy`, etc.) are installed correctly.
