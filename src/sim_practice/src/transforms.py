#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MovingTF(Node):
    def __init__(self):
        super().__init__("moving_tf")
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(.1,self.timer_callback)
        self.start_time = self.get_clock().now()
    
    def timer_callback(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "child"
        # moves frame in circle
        tf_msg.transform.translation.x = math.cos(t)
        tf_msg.transform.translation.y = math.sin(t)
        tf_msg.transform.translation.z = math.cos(t)
        # rotation
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.cos(t)
        tf_msg.transform.rotation.w = 1.0

        self.br.sendTransform(tf_msg)

def main():
    rclpy.init()
    node = MovingTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

