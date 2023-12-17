# Run . setup.sh to set up the environment
# export OPENSSL_ROOT_DIR=$(brew --prefix openssl)
# export GRAPHVIZ_DIR=$(brew --prefix graphviz)

# Add the Qt directory to the PATH and CMAKE_PREFIX_PATH
# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(brew --prefix qt@5)
# export PATH=$PATH:$(brew --prefix qt@5)/bin

# homebrew deps
echo "** Checking homebrew dependencies (this could take up to an hour)"
brew install $(cat brew_dependencies.txt) -q > /dev/null

# use venv
# python3 -m venv venv  --system-site-packages --symlinks

# detect venv
# if [ ! -d "venv" ]; then
# 	echo "** Creating venv"
# 	python3 -m venv venv  --system-site-packages --symlinks
# fi

# if the micromamba environment does not exist, create it
if [ ! -d "$HOME/.micromamba/envs/railbot" ]; then
	echo "** Creating railbot micromamba environment (up to 15 minutes)"
	# micromamba create python=3.10.8 -y -n railbot -c conda-forge  > /dev/null

	# Create a ros-humble desktop environment
	micromamba create -n railbot -c conda-forge -c robostack-staging ros-humble-desktop

	# Activate the environment
	# micromamba activate railbot
fi

echo "** Activating railbot venv environment"
micromamba activate railbot
# . venv/bin/activate
# pip3 install --upgrade pip

echo "** Installing development dependencies (up to 15 minutes)"
micromamba install -c conda-forge compilers cmake pkg-config make ninja colcon-common-extensions
# pip3 install pygraphviz \
#     --global-option=build_ext \
#     --global-option="-I$GRAPHVIZ_DIR/include" \
#     --global-option="-L$GRAPHVIZ_DIR/lib"

# pip3 install -U \
#   argcomplete catkin_pkg colcon-common-extensions coverage \
#   cryptography empy flake8 flake8-blind-except==0.1.1 flake8-builtins \
#   flake8-class-newline flake8-comprehensions flake8-deprecated \
#   flake8-docstrings flake8-import-order flake8-quotes \
#   importlib-metadata jsonschema lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces \
#   nose pep8 psutil pydocstyle pydot pyparsing==2.4.7 \
#   pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool > /dev/null

# echo "** Installing railbot dependencies (up to 15 minutes)"
# pip3 install -r requirements.txt > /dev/null

echo "** Shell Ready"
