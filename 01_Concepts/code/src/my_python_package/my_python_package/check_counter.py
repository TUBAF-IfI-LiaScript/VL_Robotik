# Schreibe einen ROS2 Subscriber, der den Count Wert eines ROS2 minimal_publishers aus dem String extrahiert
# Verwende dabei keine Objektorientierung, sondern nur Funktionen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def callback(msg):
    count = int(msg.data.split(':')[1])
    print(count)
    # Check if count is even // for testing colcon built --symlink-install
    #if count % 2 == 0:
    #   print('Even')

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('check_counter')
    node.create_subscription(String, 'annotated_int', callback, 10)
    rclpy.spin(node)
    rclpy.shutdown()
