#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
from proximity_alert_system.msg import DetectedID, EstimatedDistance

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector_node')
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # カメラからの画像を購読するサブスクライバを作成
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # QRコードのデコード結果をパブリッシュするパブリッシャを作成
        self.dist_pub = self.create_publisher(
            EstimatedDistance,
            '/estimated_distance',
            10
        )

        # detected_id トピックを購読するためのサブスクライバーを作成
        self.detected_id_sub = self.create_subscription(
            DetectedID,
            '/detected_id',
            self.detected_id_callback,
            10)
        # detected_id とその受信時刻を格納する辞書
        self.detected_info = {}
        self.detection_timeout = 5.0  # 5秒で古い検出情報を削除

        # カメラキャリブレーションの結果からのパラメータ
        self.camera_matrix = np.array([[844.95830999, 0., 679.88790111], [0., 843.00146053, 535.61399355], [0., 0., 1.]])
        self.dist_coeffs = np.array([-4.28370959e-01, 2.43630480e-01, -1.97090923e-03, 2.24499639e-04, -8.28518171e-02])

    def detected_id_callback(self, msg):
        # detected_id と現在時刻を記録
        now = self.get_clock().now()
        self.detected_info[msg.id] = now
        self.get_logger().info(f'Received detected ID: {msg.id} at time {now.to_msg()}')

    def image_callback(self, msg):
        # 古い検出情報を削除
        now = self.get_clock().now()
        self.detected_info = {id: time for id, time in self.detected_info.items() if (now - time).nanoseconds / 1e9 < self.detection_timeout}

        # ROSの画像メッセージをOpenCVの画像形式に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像を
        width = frame.shape[1]
        height = frame.shape[0]
        disp = cv2.resize(frame, (width // 4, height // 4))
        cv2.imshow('Camera Image', disp)
        cv2.waitKey(1)

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
        except:
            retval = False
        if retval:
            # QRコードが一つ以上検出された場合
            for i, info in enumerate(decoded_info):
                # デコードされた情報と対応する座標を取得
                qr_info = decoded_info[i]
                qr_points = points[i]

                # デコードされた情報と座標を出力
                print(f"QR Code data: {qr_info}")
                print(f"QR Code points: {qr_points}")

                # depth = f * s / p
                f = 1000.0
                s = 0.18
                p = abs((qr_points[1][0] + qr_points[2][0] - qr_points[3][0] - qr_points[0][0]) / 2.0) + 1e-6
                depth = f * s / p
                print(f"depth : {depth}")
 
                # depth を publish
                msg = EstimatedDistance()
                msg.detected_id = qr_info
                msg.distance = depth
                self.dist_pub.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.detected_id)

def main(args=None):
    rclpy.init(args=args)
    qr_detector_node = QRCodeDetectorNode()
    rclpy.spin(qr_detector_node)

    # シャットダウン処理
    cv2.destroyAllWindows()
    qr_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
