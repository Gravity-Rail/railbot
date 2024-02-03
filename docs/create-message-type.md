# Create a message type

Log of how I created the "Person" message type, which is emitted by the webcam_identify.py node.

1. Create Person.msg in railbot_interfaces/msg

```
string name
int id
```

2. Add the message to the CMakelists.txt in railbot_interfaces

```cmake
set(msg_files
  # ...other message files...
  "msg/Person.msg"
)
```

Now `cd $RAILBOT_WS` and run `colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY` to build the message type.