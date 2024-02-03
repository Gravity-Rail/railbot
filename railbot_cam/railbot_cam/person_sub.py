import rclpy
from rclpy.node import Node
from railbot_interfaces.msg import Person

class PersonSubscriber(Node):
  """
  Create an PersonSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('person_subscriber', namespace="railbot")

    self.subscription = self.create_subscription(
      Person,
      'people',
      self.listener_callback,
      10)

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving person message: %s' % data.name)


def main(args=None):
  rclpy.init(args=args)
  person_subscriber = PersonSubscriber()
  rclpy.spin(person_subscriber)
  person_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()