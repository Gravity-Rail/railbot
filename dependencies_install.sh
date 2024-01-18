#!/bin/bash
#
# This script configures dependencies for audio recording and play test.
# Version: 1.1
# Author: Daniel Walmsley @Mangdang
# Date: 2023-06-04

# Exit the script immediately if a command exits with a non-zero status
# set -x
set -e
# Install necessary dependencies for GPT
sudo apt update
sudo apt upgrade -y
sudo apt install gnome-terminal libcanberra-gtk-module \
    libcanberra-gtk3-module python3 python3-pip \
    portaudio19-dev ffmpeg libportaudio2 alsa-utils mpv -y

pip install -r requirements.txt
