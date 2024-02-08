## Running a Whisper example

> [!TIP] Not working yet.

This actually downloads and uses https://github.com/ros-ai/ros2_whisper from inside our workspace. I haven't integrated this yet.

Build:

```bash
pip install pyaudio
cd $RAILBOT_WS/src
git clone https://github.com/ros-ai/ros2_whisper.git
cd ../
# for CUDA-equipped machines, you can specify WHISPER_CUBLAS=On
colcon build --symlink-install --cmake-args -DWHISPER_CUBLAS=Off -DPython3_FIND_VIRTUALENV=ONLY
```

Now we can run it. Note that the first time this runs it will download a model into

```bash
ros2 launch whisper_bringup bringup.launch.py n_thread:=4
```

Once the server is running, in another terminal:

```bash
pip install pynput # only need this the first time, obviously
ros2 run whisper_demos whisper_on_key

# output
[INFO] [1707062712.358828991] [whisper_on_key]: Action server /whisper/inference found.
[INFO] [1707062712.359853898] [whisper_on_key]:

	Starting demo.
	Press ESC to exit.
	Press space to start listening.
	Press space again to stop listening.

[INFO] [1707062714.179312008] [whisper_on_key]: Requesting inference for 20 seconds...
 [INFO] [1707062714.184332127] [whisper_on_key]: Goal accepted.

 This is me testing whether it was.
 works. .
 Oh cool. Thank you for watching.
 [BLANK_AUDIO]
 So, thumbs up.
[INFO] [1707062734.818534564] [whisper_on_key]: Result: [' This is me testing whether it was.', ' works. .', ' Oh cool. Thank you for watching.', ' [BLANK_AUDIO]', ' So, thumbs up.']
[INFO] [1707062744.200433202] [whisper_on_key]: Requesting inference for 20 seconds...
 [INFO] [1707062744.203542463] [whisper_on_key]: Goal accepted.
 There we go. >> How did you look at the Android?
 out. In print's time down.
 >> Thank you very much.
 [ Silence ]
 [ Silence ]
 Tapity Tap Tap. So thank you very much.
 [BLANK_AUDIO]
 Silence!
[INFO] [1707062764.376587809] [whisper_on_key]: Goal accepted.
```

Note to self: customizing the model to use actually requires more than just modifying config.yaml. Turns out it's maybe kinda ignoring the YAML file?

Instead modify model name in `inference_node.cpp`:

```cpp
// whisper parameters
node_ptr_->declare_parameter("model_name", "medium.en"); // was base.en
```

Or maybe (TBD) the choices in whisper_server_mixin.py