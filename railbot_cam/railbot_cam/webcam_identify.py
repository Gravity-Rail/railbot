import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageIdentifier(Node):
  """
  Create an ImageIdentifier class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_identifier', namespace="railbot")

    self.subscription = self.create_subscription(
      Image,
      'video_frames',
      self.listener_callback,
      10)

    self.face_classifier = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Identifier receiving video frame')

    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    gray_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    face = self.face_classifier.detectMultiScale(
        gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
    )
    for (x, y, w, h) in face:
      cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 4)
    cv2.imshow("camera", current_frame)



    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageIdentifier()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()