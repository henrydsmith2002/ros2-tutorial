#/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs import String


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.subscription = self.create_subscription(String, "chatter", self.callback, 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()