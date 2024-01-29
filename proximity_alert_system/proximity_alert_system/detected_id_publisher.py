#!/usr/bin/env python3

import os
import subprocess
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from proximity_alert_system.msg import DetectedID

# sudo hciconfig hci0 down
# sudo hciconfig hci0 up
# sudo hciconfig hci0 leadv
# sudo hcitool -i hci0 cmd 0x08 0x0008 09 08 FF FF FF 30 30 30 30 31

class DetectedIdPublisherNode(Node):
    def __init__(self):
        super().__init__('detected_id_publisher')
        self.publisher_ = self.create_publisher(DetectedID, '/detected_id', 10)
        self.create_timer(3, self.read_ble_data)

    def read_ble_data(self):
        # ROS2パッケージのインストールディレクトリからスクリプトのパスを取得
        package_prefix = get_package_prefix('proximity_alert_system')
        receiver_path = os.path.join(package_prefix, 'lib', 'proximity_alert_system', 'receive.py')

        process = subprocess.Popen(['sudo', 'python3', receiver_path], stdout=subprocess.PIPE)
        output, _ = process.communicate()
        output_str = output.decode('utf-8').strip()
        self.get_logger().info(f"Received data: {output_str}")
        msg = DetectedID()
        msg.id = output_str
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.id)

def main(args=None):
    rclpy.init(args=args)
    node = DetectedIdPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

