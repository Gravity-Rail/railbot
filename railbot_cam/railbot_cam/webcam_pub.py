# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

# class ImageSubscriber(Node):
#     def __init__(self, name):
#         super().__init__(name)                                  # ROS2节点父类初始化
#         self.sub = self.create_subscription(
#             Image, 'image_raw', self.listener_callback, 10)     # 创建原始图像的订阅者
#         self.pub = self.create_publisher(
#             Image, 'cv_bridge_image', 10)                       # 创建cv处理之后的图像发布者
#         self.cv_bridge = CvBridge()                             # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换


#     def listener_callback(self, data):
#         image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # 将ROS的图像消息转化成OpenCV图像

#         (rows,cols,channels) = image.shape                      # 在OpenCV的显示窗口中绘制一个圆，作为标记
#         if cols > 60 and rows > 60 :
#             cv2.circle(image, (60, 60), 30, (0,0,255), -1)

#         cv2.imshow("Image window", image)                       # 使用OpenCV显示处理后的图像效果
#         cv2.waitKey(3)

#         self.pub.publish(self.cv_bridge.cv2_to_imgmsg(image, "bgr8"))   # 发布cv处理之后的图像

# def main(args=None):                                        # ROS2节点主入口main函数
#     rclpy.init(args=args)                                   # ROS2 Python接口初始化
#     node = ImageSubscriber("cv_bridge_test")                # 创建ROS2节点对象并进行初始化
#     rclpy.spin(node)                                        # 循环等待ROS2退出
#     node.destroy_node()                                     # 销毁节点对象
#     rclpy.shutdown()                                        # 关闭ROS2 Python接口

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher', namespace="railbot")

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

    # We will publish a message every 0.2 seconds, i.e. 5 frames per second
    timer_period = 0.2  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()

    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    # Display the message on the console
    self.get_logger().info('Publishing video frame')

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_publisher = ImagePublisher()

  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()