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

Now in Python code you should be able to import the message type like this:

```python
from railbot_interfaces.msg import Person

class MyNode(Node):
	# ...
	# to publish
	self.publisher = self.create_publisher(Person, 'people', 10)
	self.publisher.publish(Person(name=m.tags["name"], id=m.id))

	# to subscribe
	self.subscription = self.create_subscription(
		Person,
		'people',
		self.listener_callback,
		10)
```