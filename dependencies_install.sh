#!/bin/bash
#
# This script configures dependencies for audio recording and play test.
# Version: 1.1
# Author: Herman Ye @Mangdang
# Date: 2023-06-04

# Exit the script immediately if a command exits with a non-zero status
# set -x
set -e
# Install necessary dependencies for GPT
sudo apt update
sudo apt upgrade -y
sudo apt install gnome-terminal -y
sudo apt-get install libcanberra-gtk-module libcanberra-gtk3-module -y
sudo apt install -y python3
sudo apt install -y python3-pip
pip install pysocks
pip install requests
pip install openai==1.3
pip install depthai
pip install opencv-python
 
pip install numpy
pip install sounddevice
pip install pydub
pip install scipy
sudo apt install portaudio19-dev -y
sudo apt install ffmpeg -y

# Install dependencies for sounddevice/soundfile
sudo apt install libportaudio2 -y
sudo apt install alsa-utils -y
sudo apt install mpv -y
pip install numpy sounddevice cffi soundfile

# Check again
sudo apt update
sudo apt upgrade -y
