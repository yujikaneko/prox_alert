#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        # 解像度を設定
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # カメラフレームをROSメッセージに変換
            image_message = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            # トピックを公開
            self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

