# Creating a package

Some of these instructions are adapted from [this tutorial](https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/).

What follows is a developer log that shows how I created the railbot_cam node.

```bash
cd $YOUR_ROS2_WS
ros2 pkg create --build-type ament_python railbot_cam --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2

going to create a new package
package name: railbot_cam
destination directory: /Users/daniel/workspace/robotics/railbot
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['daniel <dan.walmsley@a8c.com>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: ['rclpy', 'image_transport', 'cv_bridge', 'sensor_msgs', 'std_msgs', 'opencv2']
creating folder ./railbot_cam
creating ./railbot_cam/package.xml
creating source folder
creating folder ./railbot_cam/railbot_cam
creating ./railbot_cam/setup.py
creating ./railbot_cam/setup.cfg
creating folder ./railbot_cam/resource
creating ./railbot_cam/resource/railbot_cam
creating ./railbot_cam/railbot_cam/__init__.py
creating folder ./railbot_cam/test
creating ./railbot_cam/test/test_copyright.py
creating ./railbot_cam/test/test_flake8.py
creating ./railbot_cam/test/test_pep257.py
```

Now I added a file to publisht the webcam frames to a ROS topic:

```bash

```

If you haven't already initialized rosdep, do this in your WS root:

```bash
rosdep init
rosdep update
```

Now install dependencies:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

Now build:

```bash
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY
```

Run:

```bash
ros2 run railbot_cam img_publisher
[INFO] [1705879269.659601359] [railbot.image_publisher]: Publishing video frame
[INFO] [1705879269.766780863] [railbot.image_publisher]: Publishing video frame
[INFO] [1705879269.860619622] [railbot.image_publisher]: Publishing video frame
[INFO] [1705879269.966986210] [railbot.image_publisher]: Publishing video frame
```

And:

```bash
ros2 run railbot_cam img_subscriber
[INFO] [1705879268.972452766] [railbot.image_subscriber]: Receiving video frame
[INFO] [1705879269.001118214] [railbot.image_subscriber]: Receiving video frame
[INFO] [1705879269.074362897] [railbot.image_subscriber]: Receiving video frame
[INFO] [1705879269.270554120] [railbot.image_subscriber]: Receiving video frame
[INFO] [1705879269.297025777] [railbot.image_subscriber]: Receiving video frame
```

By selecting the ros2 display window you should see the video frames being shown.

Now let's publish selected video frames using ChatGPT with Images.

