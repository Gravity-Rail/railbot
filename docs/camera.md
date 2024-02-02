# Camera

Set up your environment.

```bash
export RAILBOT_WS=/Users/daniel/workspace/robotics/railbot_ws
. ./env.sh
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

## Face detection with OpenCV and recognition with dlib

```bash
ros2 run railbot_cam img_identifier
```
