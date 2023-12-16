mkdir -p ~/ros2_iron/src
cd ~/ros2_iron
# get the code
vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src
# build the code
colcon build --symlink-install --packages-skip-by-dep python_qt_binding