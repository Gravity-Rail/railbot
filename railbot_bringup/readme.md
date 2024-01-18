# Launch Package for Robots using OpenAI APIs

This package is designed to be run within the ROS2 framework.

## The Basics

This file:

1. Start the parameter server
2. Start the Chat LLM service node
3. Start the audio output node
4. Start the audio input node
5. Start the robot control node

Running this launch file will enable the robot to:

- Listen and understand external language commands
- Execute external language commands
- Receive and transmit audio signals

## How to Use

Make sure you have ROS2 installed and have set up the environment.

```
ros2 launch railbot_bringup mini_pupper_launch.py
```

## Parameter Configuration

## Node Descriptions

The following describes each of the nodes included in the launch file:

1. `railbot_param_server`: This is the robot parameter server node, responsible for managing the robot's parameters.

2. `chat_llm_service`: This node provides an OpenAI-based OPENAI_ROS2 service, allowing users to control the robot through language commands. Uses LangChain so running local LLMs is also possible.

3. `audio_output`: This is the audio output node, turning the text from the LLM into spoken audio.

4. `audio_input`: This is the audio input node, turning the audio from the microphone into input for a Chat LLM.

5. `mini_pupper_robot`: This is the robot control node, used to execute the actual robot movement and control, currently on a Mini Pupper 2.