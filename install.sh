#!/bin/bash
#
# Copyright 2023 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# Main install script

set -x
set -e

trap error_check EXIT SIGHUP SIGINT
function error_check() {
  # Check the exit status of the last command
  if [ $? -ne 0 ]; then
    echo "An error or interrupted occurred. The script will now exit."
    read -n1 -r -p "Press any key to continue..." key
    echo ""
    exit 1
  fi
}

# Get directory where this script is installed
BASEDIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# Clone & build the repository
if [ -d "$HOME/gpt4_ros2_ws" ]; then
  echo "Removing existing gpt4_ros2_ws repository..."
  rm -rf "$HOME/gpt4_ros2_ws"
fi
mkdir -p $HOME/gpt4_ros2_ws/src
cd $HOME/gpt4_ros2_ws/src
git clone --depth=1 https://github.com/gravityrail/gpt4-turbo-minipupper2-ros2-humble.git gpt4_ros2

# Install necessary dependencies
cd gpt4_ros2
sudo chmod +x dependencies_install.sh
. dependencies_install.sh
cd $HOME/gpt4_ros2_ws
gpt4_ros2_ws_dir=$(pwd)
echo $gpt4_ros2_ws_dir
rosdep install --from-paths src --ignore-src -r -y
# source /opt/ros/humble/setup.bash
colcon build --symlink-install

sudo sed -i '#source $HOME/gpt4_ros2_ws/install/setup.bash#d' $HOME/.bashrc
echo "source $HOME/gpt4_ros2_ws/install/setup.bash" >>$HOME/.bashrc
. $HOME/gpt4_ros2_ws/install/setup.bash

# Ask user for GPT API_KEY
read -p "Enter your GPT API_KEY: " API_KEY
cd $gpt4_ros2_ws_dir/src/gpt4_ros2/gpt_status/gpt_status
pwd
sudo sed -i "s#<YOUR_API_KEY>#$API_KEY#" gpt_config.py
if [[ $? -eq 0 ]]; then
  echo "Add API_KEY executed successfully!"
else
  exit 1
fi
# print success message and wait for user to press any key to run the server and client
echo "Press any key to run Demo 1: Simple robot GPT call on the PC side."
read -p "If you need to run a more advanced demo, please read the readme.md file under the gpt4_ros2 package." -n1 -s
gnome-terminal --disable-factory -- bash -c 'ros2 run gpt_main gpt_ros2_server' &
gnome-terminal --disable-factory -- bash -c 'ros2 run gpt_main gpt_ros2_client' &

# Exit the script with a success status code
exit 0
