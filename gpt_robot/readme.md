This code defines a `MiniPupperRobot` class that extends the `Node` class and responsible for controlling the head behavior of a MiniPupper robot.

This code has been adapted from the excellent work by MangDang at

The status values used by the MiniPupperRobot are:

WAITING_USER_INPUT, ROBOT_ACTION, STT_PROCESSING, CHAT_LLM_PROCESSING, and TTS_PROCESSING

## Features

- Shake and nod head animations for the robot's head.
- Handles different robot states by displaying and transitioning between them.
- Ability to play a music file during certain robot states.
- Real-time state updates through the robot_behavior_callback function.
- Implements multi-threading to ensure smooth animations and music playback.
- RailbotStatusOperation class to handle GPT status.

## Usage

To use the `MiniPupperRobot` class, simply create an instance and start the node. The robot will automatically respond to the GPT status values, showing the appropriate animation and head movement depending on the robot's current status.

Below is a brief explanation of each method and their animations:

- `nod_head()`: Animates the robot's head nodding up and down.
- `shake_head()`: Animates the robot's head shaking side-to-side.
- `robot_behavior_callback()`: Continuously updates the robot's state based on the current GPT status value and executes the corresponding actions for each state.
- `play_music()`: Plays an mp3 music file during certain robot states.
- `get_real_path()`: Helper method to get the real path for the music file.

Upon executing this example, the robot will start, and depending on the different GPT status values, the robot will animate between shaking and nodding its head, and play music during some states.