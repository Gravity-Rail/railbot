<h1 align="center">RailBot</h1>

<p align="center">
    <a href="https://discord.gg/S8MRa2gK"><img alt="Discord" src="https://img.shields.io/discord/1186108372097962164?logo=discord&logoColor=white&style=flat"/></a>
    <img src="https://img.shields.io/static/v1?label=license&message=Apache&color=white&style=flat" alt="License"/>
    <br>
    <br>
    <strong>An easy way to get started with robotics and multimodal LLMs.</strong>
</p>

<br>

# Quickstart

I'm hoping that this will work on most platforms:

```bash
# this should be wherever your code lives on your workstation...
cd ~/
# clone the repo and set up dependencies
git clone https://github.com/Gravity-Rail/railbot.git
cd railbot
. ./setup.sh

# link into a "ROS workspace"
mkdir -p ~/railbot_ws/src && cd ~/railbot_ws/src
ln -s ~/railbot .
cd ../
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY

source ./install/local_setup.sh
OPENAI_API_KEY="sk-..." ros2 launch railbot_bringup mini_pupper_launch.py
```

From here, you can run standard robotics commands like `rvis2` and `rqt_graph`, and also demos like this:

```bash
source ./install/local_setup.sh
OPENAI_API_KEY="sk-..." ros2 launch railbot_bringup mini_pupper_launch.py
```

# Overview

The purpose of Railbot is to allow hackers and tinkerers to build interactive robots using the latest LLM techniques. It is in "initial commit" state with very few features beyond a simple proof-of-concept, but if you're excited to help out then we welcome patches :smiley:

After following the steps in this README you will be able to:

- Install the necessary packages to make fun interactive robots ([ROS 2](https://docs.ros.org/en/humble/index.html) and [LangChain](https://github.com/langchain-ai/langchain), MacOS and Linux supported, Windows coming soon)
- Build "workspace" which is where you customize and configure your specific robot
- Run a simple example with voice interaction and a simulated robot environment
- If you have a Mini Pupper 2, then you can deploy this to a real robot. Other robots coming soon!

Initial feature goals:

- Uses [LangChain](https://github.com/langchain-ai/langchain) with [OpenAI APIs](https://platform.openai.com/) by default, to enable simple integration of cutting edge LLM techniques.
- Built as a set of [ROS 2](https://docs.ros.org/en/humble/index.html) packages, enabling the code to run on real robots or in simulation.
- Lightweight and developer friendly, utilizing the cross-platform [micromamba](https://mamba.readthedocs.io/en/latest/user_guide/micromamba.html) tool to ensure a consistent environment across Linux, MacOS and eventually Windows.

Ideally anyone who can write basic python code on laptop on in a web-based notebook should now be able to develop ROS2 code that can run on real robot hardware or run in simulation locally.

## Why?

This project grew out of my own desire to write general-purpose code that could bring LLM-powered agent abilities, including multi-modal chat LLMs, to low-cost enthusiast robotics hardware. I am starting with the [Mini Pupper 2](https://mangdang.store/products/mini-pupper-2-ai-robot-smart-robot-quadruped-robot-educational-robot-genuine-open-source-stem-k12) because that's what I have (and I recommend it! Great kit) but I hope to expand to other platforms soon.

My pain points specifically were:
- Missing ROS2 packages in Ubuntu for AArch64 (ARM64) architecture hindered Docker development on MacOS, and x86 in emulation was too slow and hard to configure
- Migrated to x86 laptop out of frustration. Using Ubuntu in WSL2 (Windows) resulted in persistent USB issues, so...
- Then I had to install a specific version of Ubuntu on a specific laptop just to be able to develop packages for my robot running on a Raspberry Pi, and configure the GRUB bootloader etc.
- Magical parameters everywhere like `run_on_real_hardware` that you just "need to know" in order for things to work. The framework should have sensible defaults on each platform (e.g. robustly detecting "real hardware") and shouldn't expect clairvoyance from the end user.
- There has to be a better way!

Then I discovered a few magical hacks:
- [RoboStack](https://robostack.github.io/index.html) is an elegant bundling of ROS2 packages as a Conda repo for a variety of platforms. Honestly, if it weren't for the fact that I'm also trying to do LLM integration, this whole repo could be replaced with the words "Go use RoboStack".
- [Micromamba](https://mamba.readthedocs.io/en/latest/user_guide/micromamba.html) is a lightweight cross-compiled Conda-compatible package manager that allows us to load RoboStack and related dependencies in the fastest and most compatible manner possible. Unlike full Mamba, it does not have system dependencies like Python.
- The many examples provided by MangDang and others in the robotics community, including [ChatGPT integration](https://github.com/mangdangroboticsclub/chatgpt-minipupper2-ros2-humble) and underlying [ROS packages](https://github.com/mangdangroboticsclub/mini_pupper_ros). Without their working examples it would have been impossible for me to get this far. Open Source is awesome!

Once I started to make progress on these, for example being able to run the same code on my Mac, x86 Linux machine and Raspberry Pi, I realized that this was something that could be useful to others as well.

## Near Term Goals

I would like this repo to be as accessible as possible all developers, including people who have never written software before at all. Please send me your feedback on making setup simpler and more robust across as many developer environments as possible.

I am gradually trying to generalize this code as well as introduce more sophisticated integration between LLMs, multimodal, vision and audio models (TTS/STT). My goal is to find ways to manage the trade-offs between on-device, on-premise and cloud computing to maximise the utility and fun-factor of open source robots.

Ideally over time we can work together to:

- bring down perceived latency
- further to that ^^, add parallelisation abilities, e.g. fetching next part of response while responding
- bring these packages to more robot platforms
- add multimodal abilities, initially focused on Oak-D Lite and LIDAR to image uploads
- add planning abilities using function calling back to robot and virtualizing navigation across form factors
- streamline the developer experience on all developer platforms (including Windows)
- implement modules for cutting edge Robot/LLM techniques
- implement HuggingFace support
- implement support for running against local LLMs on the same network, without having to go to the cloud
- implement privacy controls for any data that is sent to the cloud, e.g. to ChatGPT-with-vision
- broaden library and language support, perhaps add Semantic Kernel or AutoGen examples
- provide tons of examples as notebooks or runnable SD card images
- provide training examples using data gathered from the device to fine tune models from HF, etc.
- implement the ability to capture and post-process data logs completely on-premise
- create new standards that enable rapid adaptation across Robot platforms, e.g. "Romoji" (my word) for describing Robot emotes, e.g. :shrug:

## Default APIs

- Chat Completions using the [OpenAI Chat Completions API](https://platform.openai.com/docs/guides/text-generation/chat-completions-api)
- stt using [OpenAI STT](https://platform.openai.com/docs/guides/speech-to-text)
- tts using [OpenAI TTS](https://platform.openai.com/docs/guides/text-to-speech)
- planned: [ChatGPT with Vision](https://platform.openai.com/docs/guides/vision)

## Why Micromamba? Why not Docker?

My personal workstation is an M2 Mac. My experience using Docker or UTM or other virtualization systems is that they have trouble talking to real hardware plugged in via USB or other interfaces. The Conda/Mamba/Micromamba packaging system provides just enough cross-platform abstraction while still compiling to the host OS and allowing for direct hardware access where necessary.

I am open to revisiting this decision as the project evolves. This was a tough one.

# Installation

Everything from here down is totally a work in progress.

## Installing ROS 2 on MacOS (may work on Linux too)

### Setup Environment

> [!TIP]
> On macOS, make sure you install [Xcode](https://apps.apple.com/app/xcode/id497799835).

> [!TIP]
> On macOS with homebrew, you can get `micromamba` by running `brew install micromamba`. This may be preferable to the default, which is executing `micro.mamba.pm/install.sh` directly.

The `setup.sh` script does the following:
1. Installs `micromamba` if necessary. This tool is used to create isolated environments and install packages.
2. Creates the `railbot` micromamba environment with desired package channels and versions (ROS 2 Humble, nodejs, cmake, colcon, rosdep, etc)
3. Activates the `railbot` micromamba environment
4. Installs packages specified in `requirements.txt`

You can run it like this:

```bash
. ./setup.sh
```

> [!TIP]
> Also, might be useful, from the [ROS 2 MacOS docs](https://docs.ros.org/en/iron/Installation/Alternatives/macOS-Development-Setup.html#disable-system-integrity-protection-sip) (TODO: Confirm this is necessary):

Confirm that your environment is working by running rvis2:

```bash
rvis2
```

### Updating environment packages

If you want to update packages later, run this:

```bash
micromamba activate railbot
micromamba update --all
```

This only applies to micromamba-managed packages. If you want to update pip packages, you'll need to do that manually.

### Running Commands

Before running ROS commands in a new shell, you need to run `micromamba activate railbot`. Once you've run that command, your environment should contain the base "underlay" for ROS:

```bash
$ env | grep ROS
ROS_DISTRO=humble
ROS_LOCALHOST_ONLY=0
ROS_PYTHON_VERSION=3
ROS_VERSION=2
ROS_OS_OVERRIDE=conda:osx
ROS_ETC_DIR=/Users/daniel/micromamba/envs/railbot/etc/ros
```

#### Run `rvis2`

`rvis2` is an [open source robotics visualization tool](https://github.com/ros2/rviz) that can display the status of your robot in 3D.

```bash
rviz2
```

#### Run `ros2`

`ros2` is a command-line tool for ROS 2.

Here's a few examples:

```bash
# list out of date packages and other possible issues
ros2 wtf
# list interfaces (like IDL) for things like goals, sensor data, etc.
ros2 interface list
# list topics that can be subscribed to which will emit or receive structured data
ros2 topic list
# run a node that emits "Hello World {N}" messages - this is included with ROS2
ros2 run demo_nodes_cpp talker
# listen for messages from the previous node
ros2 run demo_nodes_cpp listener
```

#### Run `rqt_graph`

Once you have some ROS 2 nodes running, you can visualize them using `rqt_graph`

```bash
rqt_graph
```

Try running this app, then running the `talker` example above, then click the "refresh" button. Then run the `listener` example and refresh again. You should see the nodes appearing and disappearing, including their topic subscriptions.

### Build a Workspace

In Ros 2, you run packages from a [Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), which is a specially-arranged directory that includes a shell script that you source before running the packages.

Your "underlay" is the ROS 2 installation (in our case, ROS 2 "[Humble](https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html)"). Note that MacOS is a "tier 3 platform" and so may be missing packages that are available on Linux or Windows.

The "underlay" for our workspace is the main setup.bash file for the ROS 2 installation.

```bash
# TODO THIS IS WRONG AND UNNECESSARY ON MACOS
# source /opt/ros/humble/setup.bash
```

Now you'll want to create your own workspace directory.

```bash
# change this to whatever you want, but don't forget to have it set in your
# shell when running any of these commands again.
RAILBOT_SOURCE=~/railbot

# this is your workspace directory, change to whatever you want
RAILBOT_WS=~/railbot_ws

# let's make our workspace dir
mkdir -p $RAILBOT_WS/src
cd $RAILBOT_WS/src
```

On Linux you would run `rosdep` right now, but on MacOS we don't. We use `brew` and `micromamba` (or `conda`, if you prefer) to install system-level dependencies instead. Hopefully this was handled when you ran the `. ./setup.sh` script above.

Now let's symlink our source into the workspace.

```bash
ln -s $RAILBOT_SOURCE $RAILBOT_WS/src
```

And now let's build *from the root of our workspace* (otherwise packages will be built in whatever directory you're in):

```bash
cd $RAILBOT_WS
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY
```

Some useful arguments for `colcon build`:

 * `--symlink-install` saves you from having to rebuild every time you tweak python scripts
 * `--cmake-args -DPython3_FIND_VIRTUALENV=ONLY` helps avoid errors if using micromamba on macOS, and possibly other platforms.
 * `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
 * `--event-handlers` console_direct+ shows console output while building (can otherwise be found in the log directory)

Now running `ls` in the current dir should show some additional directories alongside `railbot`:

```bash
% ls
build	install	log	src
```

> [!TIP]
> Every time you rebuild using `colcon build`, be sure to re-run `. ./install/local_setup.zsh`, otherwise you may get missing package errors when trying to launch a package.

### Running an audio chat example

```bash
source ./install/local_setup.sh
OPENAI_API_KEY="sk-..." ros2 launch railbot_bringup mini_pupper_launch.py
```

Now when you see `Starting audio recording...` try saying a few words. After capturing a few seconds of audio, it sends it to OpenAI to be converted into text. Then, it sends the Text to the Chat Completions endpoint to compute a response. Finally, it converts the response text back into speech using _another_ API. No wonder it takes so long to reply!

# Usage

## Demo: OpenAI-based interaction on Mini Pupper 2

After configuring everything, run the below commands on Mini Pupper 2:

```bash
# Terminal 1 Bringup mini pupper
. ~/ros2_ws/install/setup.bash
ros2 launch railbot_bringup bringup.launch.py
```

```bash
# Terminal 2 Bringup RailBot layer
export OPENAI_API_KEY="sk-..."
ros2 launch railbot_bringup mini_pupper_launch.py
```

Huge thanks to MangDang Robotics Club for the [high quality examples for the Mini Pupper 2](https://github.com/mangdangroboticsclub/chatgpt-minipupper2-ros2-humble) which made all this possible. Go buy their robots!
