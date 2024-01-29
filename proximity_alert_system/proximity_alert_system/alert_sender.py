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

class AlertSenderNode(Node):
    def __init__(self):
        super().__init__('detected_id_publisher')
        self.dist_sub = self.create_subscription(
            DetectedID,
            '/detected_id',
            self.dist_callback,
            10
        )
        subprocess.run(['sudo', 'hciconfig', 'hci0', 'down'], check=True)
        subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], check=True)
        subprocess.run(['sudo', 'hciconfig', 'hci0', 'leadv'], check=True)
        subprocess.run(['sudo', 'hcitool', '-i', 'hci0', 'cmd', '0x08', '0x0008', '00'], check=True)

    def dist_callback(self, msg):
        packet = msg.id + str(int(msg.distance * 1000.0))
        print(packet)


def main(args=None):
    rclpy.init(args=args)
    node = AlertSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

