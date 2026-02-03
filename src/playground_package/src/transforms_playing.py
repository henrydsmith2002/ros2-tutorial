#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    # ROS uses (x, y, z, w)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

class DynamicTF(Node):
    def __init__(self):
        super().__init__('dynamic_tf')
        self.br = TransformBroadcaster(self)
        self.t = 0
        self.timer = self.create_timer(.02,self.tick) #50 Hz

    def tick(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'moving_frame'
        # translate
        tf.transform.translation.x = .5
        tf.transform.translation.y = .0
        tf.transform.translation.z = .2
        # rotate in yaw over time 
        yaw = self.t
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        
        self.br.sendTransform(tf)
        self.t += .02

def main():
    rclpy.init()
    node = DynamicTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
