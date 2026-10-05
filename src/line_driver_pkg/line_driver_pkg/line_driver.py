#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw',
                                                self.callback, 1)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        y_fraction = 0.9
        y = int(cv_image.shape[0] * y_fraction)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        row = gray[y]

        dark_threshold = 100
        dark_pixels = np.where(row < dark_threshold)[0]

        cmd = Twist()

        if len(dark_pixels) > 0:
            center_x = int((dark_pixels[0] + dark_pixels[-1]) / 2)
            image_center = cv_image.shape[1] / 2

            error = center_x - image_center

            cmd.linear.x = 0.1
            cmd.angular.z = -0.001 * error

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    print("Starting...")
    rclpy.init(args=args)
    lf = LineFollower()
    try:
        rclpy.spin(lf)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
        lf.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
