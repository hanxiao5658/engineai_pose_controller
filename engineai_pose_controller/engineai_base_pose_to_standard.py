#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EngineAI MuJoCo base pose -> standard ROS2 PoseStamped.

ROS2 Humble, inside the EngineAI simulation container.

Input : /simulation/base_link           interface_protocol/msg/LinkInfo
        LinkInfo fields: header, pose (geometry_msgs/Pose), twist.
Output: /my_ros2/simulation/base_link   geometry_msgs/msg/PoseStamped (50 Hz)

The SDK's LinkInfo.pose is already in ROS (x,y,z,w) quaternion order.
We do not interpret the orientation as Euler angles or swap components.

IMPORTANT: This conversion does NOT transform coordinate frames. The input
must describe LINK_BASE in the MuJoCo *world* frame, aligned to Gazebo world.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from interface_protocol.msg import LinkInfo


class EngineAIBasePoseToStandard(Node):
    def __init__(self):
        super().__init__('engineai_base_pose_to_standard')
        self.declare_parameter('input_topic', '/simulation/base_link')
        self.declare_parameter('output_topic', '/my_ros2/simulation/base_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('stale_timeout', 0.5)
        # The SDK's simulated_base_state_in_world is expressed in world.
        # Used only if the source Header.frame_id is blank.
        self.declare_parameter('fallback_frame_id', 'world')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.fallback_frame_id = str(self.get_parameter('fallback_frame_id').value)
        if not (math.isfinite(rate) and rate > 0):
            raise ValueError('publish_rate must be > 0')
        if not (math.isfinite(self.stale_timeout) and self.stale_timeout > 0):
            raise ValueError('stale_timeout must be > 0')

        # BEST_EFFORT subscriptions can receive BEST_EFFORT or RELIABLE publishers.
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        # A RELIABLE standard topic is convenient for the Foxy ros1_bridge.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self._latest: Optional[LinkInfo] = None
        self._last_receive_wall: Optional[float] = None
        self._warned_frame = False
        self._publisher = self.create_publisher(PoseStamped, output_topic, output_qos)
        self._subscriber = self.create_subscription(
            LinkInfo, input_topic, self._on_link_info, input_qos)
        self._timer = self.create_timer(1.0 / rate, self._publish_latest)
        self.get_logger().info(
            f'Base pose converter: {input_topic} -> {output_topic}, {rate:.1f} Hz')

    def _on_link_info(self, msg: LinkInfo) -> None:
        self._latest = msg
        self._last_receive_wall = time.monotonic()

    def _publish_latest(self) -> None:
        src = self._latest
        if src is None or self._last_receive_wall is None:
            return
        if time.monotonic() - self._last_receive_wall > self.stale_timeout:
            return  # Avoid replaying an old pose after MuJoCo stops.

        p = src.pose.position
        q = src.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(v)) for v in vals):
            self.get_logger().warning('Invalid (NaN/Inf) base pose; skipped')
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-8:
            self.get_logger().warning('Zero quaternion; skipped')
            return

        out = PoseStamped()
        out.header.stamp = src.header.stamp
        out.header.frame_id = src.header.frame_id or self.fallback_frame_id
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()

        if out.header.frame_id != 'world' and not self._warned_frame:
            self.get_logger().warning(
                f'Source frame_id={out.header.frame_id!r}. This node does not '
                'transform frames. Confirm alignment with Gazebo world.')
            self._warned_frame = True

        out.pose.position.x = float(p.x)
        out.pose.position.y = float(p.y)
        out.pose.position.z = float(p.z)
        # Geometry quaternion component order is x,y,z,w (no Euler conversion).
        out.pose.orientation.x = float(q.x / norm)
        out.pose.orientation.y = float(q.y / norm)
        out.pose.orientation.z = float(q.z / norm)
        out.pose.orientation.w = float(q.w / norm)
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EngineAIBasePoseToStandard()
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
