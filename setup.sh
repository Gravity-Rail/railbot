# if micromamba does not exist, install it
if ! command -v micromamba &> /dev/null; then
	echo "** Installing micromamba"
	# nope, turns out no support for Raspberry Pi
	# "${SHELL}" <(curl -L micro.mamba.pm/install.sh)
	# btw this next command doesn't actually work on 32-bit Pi either since CPU gets reported as arm71 instead of aarch64
	# make sure you're running the 64-bit Pi os!
	curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
	bash Miniforge3-$(uname)-$(uname -m).sh
fi

# if the micromamba environment does not exist, create it
if [ ! -d "$HOME/.micromamba/envs/railbot" ]; then
	echo "** Creating railbot micromamba environment (up to 15 minutes)"
	micromamba create -n railbot -c conda-forge -c robostack-staging -c robostack-experimental \
		ros-humble-desktop rosdep nodejs==18.9.1 compilers cmake pkg-config make ninja colcon-common-extensions
fi

echo "** Activating railbot venv environment"
micromamba activate railbot

echo "** Installing pip dependencies"
pip install -r requirements.txt

echo "** Shell Ready"
