#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert bridged ROS1 body-frame Twist commands to EngineAI BodyVelCmd.

ROS1 navigation controller publishes:
    /my_ros1/motion/body_vel_cmd   geometry_msgs/Twist
    linear.x  : forward velocity, m/s
    linear.y  : leftward velocity, m/s
    angular.z : CCW yaw velocity, rad/s

ros1_bridge preserves the topic name when forwarding it to ROS2.
This node (ROS2 Humble) receives that standard Twist and publishes:
    /motion/body_vel_cmd          interface_protocol/msg/BodyVelCmd

This is a message-format adapter only. It has no watchdog, output timer,
velocity scaling, clamping, or pose control. One input -> one output.
"""

import rclpy
from geometry_msgs.msg import Twist
from interface_protocol.msg import BodyVelCmd
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)


class BodyVelTwistToVendor(Node):
    def __init__(self):
        super().__init__('body_vel_twist_to_vendor')

        self.declare_parameter('input_topic', '/my_ros1/motion/body_vel_cmd')
        self.declare_parameter('output_topic', '/motion/body_vel_cmd')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        # BEST_EFFORT subscription can accept both RELIABLE and BEST_EFFORT
        # ros1_bridge publishers. Output uses the vendor's default QoS.
        input_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(BodyVelCmd, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, input_topic, self.on_twist, input_qos
        )

        self.get_logger().info(
            f'Body velocity adapter ready: {input_topic} (Twist) -> '
            f'{output_topic} (BodyVelCmd); frame_id=body'
        )

    def on_twist(self, msg: Twist):
        out = BodyVelCmd()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'body'

        # Both inputs and outputs are expressed in the ROBOT BODY frame.
        # ROS body axes: +X forward, +Y left, +Z up; +yaw CCW.
        out.linear_velocity = [float(msg.linear.x), float(msg.linear.y)]
        out.yaw_velocity = float(msg.angular.z)

        # Twist.linear.z and Twist.angular.x/y are intentionally unused.
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BodyVelTwistToVendor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
