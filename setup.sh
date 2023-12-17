# Run . setup.sh to set up the environment

# homebrew deps
echo "** Checking homebrew dependencies (this could take up to an hour)"
brew install $(cat brew_dependencies.txt) -q > /dev/null

# if the micromamba environment does not exist, create it
if [ ! -d "$HOME/.micromamba/envs/railbot" ]; then
	echo "** Creating railbot micromamba environment (up to 15 minutes)"
	micromamba create -n railbot -c conda-forge -c robostack-staging -c robostack-experimental \
		ros-humble-desktop rosdep nodejs==18.9.1 compilers cmake pkg-config make ninja colcon-common-extensions
fi

echo "** Activating railbot venv environment"
micromamba activate railbot
pip install -r requirements.txt

echo "** Shell Ready"
