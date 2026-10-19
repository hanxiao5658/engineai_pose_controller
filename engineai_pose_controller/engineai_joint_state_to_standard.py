#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert EngineAI custom joint state to standard ROS2 sensor_msgs/JointState.

Subscribe:
    /hardware/joint_state
        interface_protocol/msg/JointState
        fields:
            header
            float64[] position
            float64[] velocity
            float64[] torque

Publish:
    /my_ros2/hardware/joint_state
        sensor_msgs/msg/JointState
        fields:
            header
            string[] name
            float64[] position
            float64[] velocity
            float64[] effort

Why this node exists:
    EngineAI's JointState does not include joint names and uses "torque".
    Standard ROS JointState requires joint names and uses "effort".
    After converting to sensor_msgs/msg/JointState, ros1_bridge can bridge it
    to ROS1 without recompiling custom interface_protocol messages.
"""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from interface_protocol.msg import JointState as EngineAIJointState
from sensor_msgs.msg import JointState as StandardJointState


DEFAULT_T800_JOINT_NAMES = [
    "J00_HIP_PITCH_L",
    "J01_HIP_ROLL_L",
    "J02_HIP_YAW_L",
    "J03_KNEE_PITCH_L",
    "J04_ANKLE_PITCH_L",
    "J05_ANKLE_ROLL_L",
    "J06_HIP_PITCH_R",
    "J07_HIP_ROLL_R",
    "J08_HIP_YAW_R",
    "J09_KNEE_PITCH_R",
    "J10_ANKLE_PITCH_R",
    "J11_ANKLE_ROLL_R",
    "J12_TORSO_YAW",
    "J13_SHOULDER_PITCH_L",
    "J14_SHOULDER_ROLL_L",
    "J15_SHOULDER_YAW_L",
    "J16_ELBOW_PITCH_L",
    "J17_ELBOW_YAW_L",
    "J18_SHOULDER_PITCH_R",
    "J19_SHOULDER_ROLL_R",
    "J20_SHOULDER_YAW_R",
    "J21_ELBOW_PITCH_R",
    "J22_ELBOW_YAW_R",
    "J23_HEAD_PITCH",
    "J24_HEAD_YAW",
]


class EngineAIJointStateToStandard(Node):
    """
    Convert /hardware/joint_state to /my_ros2/hardware/joint_state at fixed rate.

    Important:
        EngineAI message only has arrays and does not carry joint names.
        Therefore the order of DEFAULT_T800_JOINT_NAMES must match the order used
        by EngineAI /hardware/joint_state.
    """

    def __init__(self):
        super().__init__("engineai_joint_state_to_standard")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topic", "/hardware/joint_state")
        self.declare_parameter("output_topic", "/my_ros2/hardware/joint_state")
        self.declare_parameter("publish_rate", 50.0)

        # If True, subscribe with BEST_EFFORT.
        # This is often safer for robot state feedback topics.
        self.declare_parameter("use_best_effort_sub", True)

        # If True, print one warning when received joint array length is not 25.
        self.declare_parameter("warn_length_mismatch", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.use_best_effort_sub = bool(self.get_parameter("use_best_effort_sub").value)
        self.warn_length_mismatch = bool(self.get_parameter("warn_length_mismatch").value)

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive.")

        self.joint_names: List[str] = list(DEFAULT_T800_JOINT_NAMES)

        # Latest received EngineAI joint state.
        self.latest_msg: Optional[EngineAIJointState] = None
        self.has_warned_length_mismatch = False

        # ---------------- QoS ----------------
        if self.use_best_effort_sub:
            input_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
        else:
            input_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

        # Publisher uses normal reliable QoS.
        # This is friendly to ros1_bridge and downstream tools.
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---------------- ROS interfaces ----------------
        self.sub = self.create_subscription(
            EngineAIJointState,
            self.input_topic,
            self.joint_state_callback,
            input_qos,
        )

        self.pub = self.create_publisher(
            StandardJointState,
            self.output_topic,
            output_qos,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_latest_joint_state,
        )

        self.get_logger().info(
            "EngineAIJointStateToStandard started. "
            f"input={self.input_topic}, "
            f"output={self.output_topic}, "
            f"publish_rate={self.publish_rate:.1f} Hz, "
            f"use_best_effort_sub={self.use_best_effort_sub}"
        )

    def joint_state_callback(self, msg: EngineAIJointState) -> None:
        """Store latest EngineAI joint state. Actual publish is done by 50 Hz timer."""
        self.latest_msg = msg

    def publish_latest_joint_state(self) -> None:
        """Publish latest joint state as standard sensor_msgs/JointState."""
        if self.latest_msg is None:
            return

        src = self.latest_msg

        num_names = len(self.joint_names)
        num_pos = len(src.position)
        num_vel = len(src.velocity)
        num_torque = len(src.torque)

        # Position is the minimum required field for our Gazebo sync workflow.
        # We only publish joints that have a valid position and known name.
        n = min(num_names, num_pos)

        if n <= 0:
            return

        if (
            self.warn_length_mismatch
            and not self.has_warned_length_mismatch
            and num_pos != num_names
        ):
            self.get_logger().warn(
                "Joint count mismatch: "
                f"received position length={num_pos}, "
                f"default T800 joint name length={num_names}. "
                f"Publishing first {n} joints. "
                "Please confirm EngineAI joint order and joint count."
            )
            self.has_warned_length_mismatch = True

        out = StandardJointState()

        # Preserve source timestamp if available.
        # This helps downstream synchronization.
        out.header = src.header

        # If upstream header stamp is zero, use current ROS time.
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()

        # Optional but useful for downstream tools.
        # Keep it simple for bridge / Gazebo usage.
        if not out.header.frame_id:
            out.header.frame_id = "t800"

        out.name = self.joint_names[:n]
        out.position = list(src.position[:n])

        # Standard JointState allows velocity/effort to be empty,
        # but if provided, their lengths should match name/position.
        if num_vel >= n:
            out.velocity = list(src.velocity[:n])
        else:
            out.velocity = []

        if num_torque >= n:
            out.effort = list(src.torque[:n])
        else:
            out.effort = []

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EngineAIJointStateToStandard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

