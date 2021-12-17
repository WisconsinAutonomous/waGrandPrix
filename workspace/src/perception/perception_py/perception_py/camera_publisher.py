#
# MIT License
#
# Copyright (c) 2018-2022 Wisconsin Autonomous
#
# See https://wa.wisc.edu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["/camera/image"] = self.create_publisher(
            Image, "/camera/image", 10)

        # Periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Parameter declarations
        # Get the input for the opencv2 VideoCapture
        self.declare_parameter('/camera/video-capture', '0')

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        video_capture = rclpy.parameter.Parameter(
            '/camera/video-capture', rclpy.Parameter.Type.STRING)
        self.cap = cv2.VideoCapture(video_capture)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def timer_callback(self):
        """
        Method will publish a sensor_msgs.Image message at a fixed rate
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_handles["/camera/image"].publish(
                self.br.cv2_to_imgmsg(frame))

            # Display the message on the console
            self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
