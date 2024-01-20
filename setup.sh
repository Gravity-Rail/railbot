#!/bin/bash

# if micromamba does not exist, install it
if ! type micromamba &> /dev/null; then
	echo "** Installing micromamba"
	# make sure you're running the 64-bit Pi os!

	# alt: miniforge / mamba
	# curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
	# bash Miniforge3-$(uname)-$(uname -m).sh -b

	"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
	. $HOME/.bashrc
fi

# if the micromamba environment does not exist, create it
if [ ! -d "$HOME/.micromamba/envs/railbot" ]; then
	echo "** Creating railbot micromamba environment (up to 15 minutes)"
	micromamba create -n railbot -c conda-forge -c robostack-staging -c robostack-experimental \
		ros-humble-desktop rosdep portaudio nodejs==18.9.1 compilers cmake pkg-config make ninja colcon-common-extensions
fi

# if apt is available, use apt install to install linux dependencies
if type apt &> /dev/null; then
	echo "** Installing linux dependencies"
	sudo apt update
	sudo apt upgrade -y
	sudo apt install gnome-terminal libcanberra-gtk-module \
		libcanberra-gtk3-module \
		ffmpeg mpv alsa-utils -y
fi

# pip install -r requirements.txt

echo "** Activating railbot micromamba environment"
micromamba activate railbot

echo "** Installing pip dependencies"
pip install -r requirements.txt

# ask the user if they want to install development dependencies
read -p "Do you want to install development dependencies? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
	echo "** Installing development dependencies"
	pip install -r requirements-dev.txt
fi

echo "** Shell Ready"
