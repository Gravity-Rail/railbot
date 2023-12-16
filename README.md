# Forked from MangDang Robotics Club

ROS2 Humble modules forked from [MangDang Robotics Club example for the Mini Pupper 2](https://github.com/mangdangroboticsclub/chatgpt-minipupper2-ros2-humble).

# Overview

The purpose of this repo is to give as many people as possible the ability to write and package code that can be run on robots, with a focus on integrating cloud LLMs with embedded code and models for the purposes of learning and entertainment.

I want this code to be as accessible as possible. I would love to get feedback on making setup simpler and more robust across as many developer environments as possible. I want to make it possible to become a robotics expert without ever needing anything more than a cheap Linux, Windows or Mac computer.

    I am gradually trying to generalize this code as well as introduce more sophisticated integration between LLMs, multimodal, vision and audio models (TTS/STT). My goal is to find ways to manage the trade-offs between on-device, on-premise and cloud computing to maximise the utility of these robots without requiring radical new technologies or approaches.

One way to think about it might be "LangChain for robots" - a place where people can quickly implement new techniques and get feedback on them.

- stt using https://platform.openai.com/docs/guides/speech-to-text
- tts using https://platform.openai.com/docs/guides/text-to-speech

Later, on-device audio:
- stt using https://github.com/ros-ai/ros2_whisper?
- tts using https://github.com/rhasspy/piper

Also, video / photos:
- OpenCV + Oak D-Lite samples
- integration with multi-modal LLMs like Gemini and GPT-4 Turbo

Also, motion:
- navigation and interaction using LLM function calling

# Summary

How it works on my Mini Pupper 2:

your voice ---> Mini Pupper 2 record by Mic x2 ---> translate voice to text by OpenAI STT service ---> chatGPT API ---> translate text to voice by OpenAI TTS service ---> Mini Pupper 2 voice Playback & Movement & emotion.

# Installation

## Installing ROS 2 on MacOS

### Setup Environment

Assumes a working [homebrew](https://brew.sh/) and micromamba environment. Also, make sure you install [Xcode](https://apps.apple.com/app/xcode/id497799835).

Set up your shell:

```bash
. ./setup.sh
```

Also, might be useful, from the [ROS 2 MacOS docs](https://docs.ros.org/en/iron/Installation/Alternatives/macOS-Development-Setup.html#disable-system-integrity-protection-sip):

> macOS/OS X versions >=10.11 have System Integrity Protection enabled by default. So that SIP doesn’t prevent processes from inheriting dynamic linker environment variables, such as DYLD_LIBRARY_PATH, you’ll need to disable it following [these instructions](https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html).

### Updating Packages

```bash
micromamba update --all
```

### Run rvis

Before running rvis or any other commands, first run `micromamba activate marvin`.

Then, to run rvis2:

```bash
rviz2
```

### Using JupyterROS

Adapted from https://github.com/RoboStack/jupyter-ros

JupyterROS allows you to prototype robotics applications from a familiar Python notebook interface, including
interactive 3D graphics.

```bash
export NODE_OPTIONS=--openssl-legacy-provider # necessary to avoid OpenSSL errors
micromamba install -c conda-forge nodejs==18.9.1
pip install jupyter jupyterlab==3.6 bqplot pyyaml ipywidgets==7.8.1 ipycanvas pymongo sidecar roslibpy
pip install git+https://github.com/RoboStack/jupyter-ros.git

jupyter nbextension install --py --symlink --sys-prefix jupyros
jupyter labextension install @jupyter-widgets/jupyterlab-manager
jupyter labextension install @jupyter-widgets/jupyterlab-sidecar

jupyter nbextension enable --py --sys-prefix jupyros
jupyter labextension enable @jupyter-widgets/jupyterlab-manager
jupyter labextension enable @jupyter-widgets/jupyterlab-sidecar
```

Now to launch:

```bash
jupyter-lab notebooks/ROS2_Keyboard_Input.ipynb
```

When you click in the small black square and press the arrow keys on your keyboard, you should see the icon change to reflect the pressed key.

## Simulated Mode

To install standalone on a PC you'll need WSL 2 with Ubuntu 20.04 LTS, or actual Ubuntu.

```bash
# make a wordspace directory for building our ROS2 package
cd ~
mkdir -p marvin_ros2_ws/src
cd marvin_ros2_ws/src

# check out the code
git clone git@github.com:Gravity-Rail/marvin
cd marvin

# install Python dependencies
. dependencies_install.sh

# install ROS dependencies
cd ~/marvin_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# build the package
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Modify your ~/.bashrc to include the following lines:

```bash
. $HOME/marvin_ros2_ws/install/setup.bash
export OPENAI_API_KEY="..."
```

Then, to run it:

```bash
bash -c 'ros2 run gpt_main gpt_ros2_server'
```

In another terminal:

```bash
bash -c 'ros2 run gpt_main gpt_ros2_client'
```

## One-click Installation

Mini Pupper 2 and Ubuntu 22.04 + ROS 2 Humble is required. Please follow the installation document [here](https://github.com/mangdangroboticsclub/mini_pupper_ros )

To install with one command, connect to your Mini Pupper 2, be certain to tell ssh to allow X11 forwarding if you want to run demo 1.

```bash
ssh -o ForwardX11=yes ubuntu@<Your Mini Pupper 2 IP address>
```

and then run the following command:

```bash
wget -O $HOME/install.sh https://raw.githubusercontent.com/Gravity-Rail/marvin/main/install.sh && sudo chmod +x $HOME/install.sh && bash $HOME/install.sh && rm $HOME/install.sh
```

After the one-click Installation, `demo 1 Simple robot GPT call on the PC side` will run automatically, if you want to run other demos, please modify the configuration file according to Step4 of Manual Installation

![Mini Pupper 2](imgs/MiniPupper.GPT.PCDemo.png)


## Manual Installation

If you want to install manually, follow the steps below.

### Step 1: Clone the repo

```bash
cd <your_ws>/src
git clone https://github.com/Gravity-Rail/marvin.git
```

### Step 2: Install dependencies

```bash
cd <your_ws>/src/gpt4-turbo-minipupper2-ros2-humble
sudo chmod +x dependencies_install.sh
. dependencies_install.sh # Install dependencies
```

### Step 3: Build the repo

```bash
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Step 4: Configuration

To use the gpt4_ros2 package, follow these steps:

#### 4.2 Set up OpenAI API
1. Create an account on [OpenAI](https://platform.openai.com).
2. Click on the user icon in the upper-right corner.
3. Click `View API keys`.
4. Click `Create new secret key`.
5. Enter a `name` and click `Create secret key`.
6. Copy your secret key and save it securely.

#### 4.3 Configure and build the package
1. Navigate to `<your_ws>/src/gpt4-turbo-minipupper2-ros2-humble/gpt_status/gpt_status/gpt_config.py`.
```bash
cd <your_ws>/src/gpt4-turbo-minipupper2-ros2-humble/gpt_status/gpt_status
```
2. Set your desired configurations, such as the GPT-4 or GPT-3.5-turbo model, system_prompt, and other attributes. Fill in the relevant configuration details for OpenAI that you obtained earlier.
```bash
sudo nano gpt_config.py
```

#### 4.4 Modify the gpt_robot package code [optional]
If you wish to use GPT for your own robots, modify the contents of the gpt_robot package, which configures physical or virtual robots.

We encourage you to customize the GPTConfig class to tailor the functionality of this ROS2 wrapper for GPT-4 and ChatGPT (GPT-3.5) according to your specific needs. To do this, simply modify the values in the code snippet to suit your requirements:

GPT-4 is currently in a limited beta and only accessible to those who have been granted access. Please join the [waitlist](https://openai.com/waitlist/gpt-4-api) to get access when capacity is available.

Feel free to adjust the parameters, such as temperature, max_tokens, and top_p, to influence the behavior of the GPT model. You can also customize the system_prompt and user_prompt strings to create unique and engaging interactions with your robot.

By personalizing these settings, you can create a one-of-a-kind experience tailored to your specific robotic application. Enjoy experimenting and discovering new possibilities!


# Usage

## Demo 1: Simple GPT call on the PC

If you want to simply try this service, configure your OpenAI API and system_prompt in `gpt_status/gpt_config.py`. Then, try:

```bash
# Terminal 1
ros2 run gpt_main gpt_ros2_server
```

```bash
# Terminal 2
ros2 run gpt_main gpt_ros2_client
```

## Demo 2: GPT service on Mini Pupper 2

After configuring everything, run the below commands on Mini Pupper 2:
```bash
# Terminal 1 Bringup mini pupper
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```
```bash
# Terminal 2 Bringup GPT
ros2 launch gpt_bringup gpt_bringup_launch.py mini_pupper:=True
```

# License
This project is licensed under the Apache-2.0 License.
```
Copyright 2023 Mangdang
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
