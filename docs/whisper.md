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
```

Note to self: customizing the model to use actually requires more than just modifying config.yaml. Turns out it's maybe kinda ignoring the YAML file?

Instead modify model name in `inference_node.cpp`:

```cpp
// whisper parameters
node_ptr_->declare_parameter("model_name", "medium.en"); // was base.en
```

Or maybe (TBD) the choices in whisper_server_mixin.py