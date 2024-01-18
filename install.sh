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

ROS2_WS_DIR="$HOME/railbot_ros2_ws"

# Clone & build the repository
if [ -d ROS2_WS_DIR ]; then
  echo "Removing existing $ROS2_WS_DIR repository..."
  rm -rf $ROS2_WS_DIR
fi
mkdir -p $ROS2_WS_DIR/src
cd $ROS2_WS_DIR/src
git clone --depth=1 https://github.com/Gravity-Rail/railbot.git

# Install necessary dependencies
cd railbot
sudo chmod +x dependencies_install.sh
. dependencies_install.sh
cd $ROS2_WS_DIR
echo $ROS2_WS_DIR
rosdep install --from-paths src --ignore-src -r -y
# source /opt/ros/humble/setup.bash
colcon build --symlink-install

sudo sed -i "#source $ROS2_WS_DIR/install/setup.bash#d" ~/.bashrc
echo "source $ROS2_WS_DIR/install/setup.bash" >>~/.bashrc
. $ROS2_WS_DIR/install/setup.bash

# Ask user for GPT API_KEY
read -p "Enter your GPT API_KEY: " API_KEY
cd $ROS2_WS_DIR/src/railbot/railbot_status/railbot_status
pwd
sudo sed -i "s#<YOUR_API_KEY>#$API_KEY#" gpt_config.py
if [[ $? -eq 0 ]]; then
  echo "Add API_KEY executed successfully!"
else
  exit 1
fi
# print success message and wait for user to press any key to run the server and client
echo "Press any key to run Demo 1: Simple robot GPT call on the PC side."
read -p "If you need to run a more advanced demo, please read the readme.md file under the railbot package." -n1 -s
gnome-terminal --disable-factory -- bash -c 'ros2 run railbot_main gpt_ros2_server' &
gnome-terminal --disable-factory -- bash -c 'ros2 run railbot_main gpt_ros2_client' &

# Exit the script with a success status code
exit 0
