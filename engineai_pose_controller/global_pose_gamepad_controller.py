#!/usr/bin/env python3
import os
import sys
import math
import time
from typing import Optional

# 让 Python 能找到 EngineAI 虚拟手柄使用的 lcm_msgs
SDK_DIR = os.environ.get(
    "ENGINEAI_ROBOTICS_DIR",
    os.path.expanduser("~/workspaces/engineai_robotics_native_sdk"),
)
VIRTUAL_GAMEPAD_DIR = os.path.join(SDK_DIR, "tools", "virtual_gamepad")
if VIRTUAL_GAMEPAD_DIR not in sys.path:
    sys.path.insert(0, VIRTUAL_GAMEPAD_DIR)

import lcm
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from interface_protocol.msg import LinkInfo
from lcm_msgs.data import GamepadKeys


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GlobalPoseGamepadController(Node):
    """
    Subscribe:
      /simulation/base_link          interface_protocol/msg/LinkInfo
      /target_pose_global_one        geometry_msgs/msg/PoseStamped

    Output:
      LCM virtual_gamepad/gamepad_keys
    """

    def __init__(self):
        super().__init__("global_pose_gamepad_controller")

        # ---------------- Parameters ----------------
        self.declare_parameter("current_pose_topic", "/simulation/base_link")
        self.declare_parameter("target_pose_topic", "/target_pose_global_one")

        self.declare_parameter("lcm_url", "udpm://239.255.76.67:7667?ttl=0")
        self.declare_parameter("lcm_channel", "virtual_gamepad/gamepad_keys")

        self.declare_parameter("control_rate", 20.0)

        # P 控制增益
        self.declare_parameter("kp_forward", 0.4)
        self.declare_parameter("kp_lateral", 0.4)
        self.declare_parameter("kp_yaw", 0.5)

        # 摇杆限幅
        self.declare_parameter("max_forward", 0.9)
        self.declare_parameter("max_lateral", 0.20)
        self.declare_parameter("max_turn", 0.9)

        # 到点阈值
        self.declare_parameter("position_tolerance", 0.10)
        self.declare_parameter("yaw_tolerance", 0.10)
        self.declare_parameter("deadband", 0.03)

        # 坐标/方向符号修正
        # left_stick_x 正数：前进
        # left_stick_y 正数：向右
        # right_stick_y 正数：向右旋转
        self.declare_parameter("forward_sign", 1.0)
        self.declare_parameter("lateral_sign", 1.0)
        self.declare_parameter("yaw_sign", -1.0)

        # 如果一段时间没有收到目标点，是否输出 0 摇杆
        self.declare_parameter("target_timeout", 300.0)

        # debug
        self.declare_parameter("debug_print", True)
        self.declare_parameter("debug_period", 0.2)

        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)

        self.lcm_url = str(self.get_parameter("lcm_url").value)
        self.lcm_channel = str(self.get_parameter("lcm_channel").value)

        self.control_rate = float(self.get_parameter("control_rate").value)

        self.kp_forward = float(self.get_parameter("kp_forward").value)
        self.kp_lateral = float(self.get_parameter("kp_lateral").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)

        self.max_forward = abs(float(self.get_parameter("max_forward").value))
        self.max_lateral = abs(float(self.get_parameter("max_lateral").value))
        self.max_turn = abs(float(self.get_parameter("max_turn").value))

        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.deadband = float(self.get_parameter("deadband").value)

        self.forward_sign = float(self.get_parameter("forward_sign").value)
        self.lateral_sign = float(self.get_parameter("lateral_sign").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)

        self.target_timeout = float(self.get_parameter("target_timeout").value)

        self.debug_print = bool(self.get_parameter("debug_print").value)
        self.debug_period = float(self.get_parameter("debug_period").value)
        self.last_debug_time = 0.0

        # ---------------- State ----------------
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.last_current_pose_time = 0.0

        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.last_target_time = 0.0

        self.reached = False

        # ---------------- LCM ----------------
        self.lcm_handle = lcm.LCM(self.lcm_url)

        # ---------------- ROS2 subscriptions ----------------
        # /simulation/base_link 是我们自己从 SDK 中桥接出来的仿真真值。
        # 它的消息类型 LinkInfo 内部包含 header、pose、twist。
        # 之前你遇到 QoS 不兼容，所以这里使用 BEST_EFFORT。
        current_pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.current_pose_sub = self.create_subscription(
            LinkInfo,
            self.current_pose_topic,
            self.current_pose_callback,
            current_pose_qos,
        )

        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_pose_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
        )

        self.get_logger().info(
            f"GlobalPoseGamepadController started. "
            f"current_pose_topic={self.current_pose_topic}, "
            f"target_pose_topic={self.target_pose_topic}, "
            f"lcm_channel={self.lcm_channel}"
        )

    def current_pose_callback(self, msg: LinkInfo):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        q = msg.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        self.last_current_pose_time = time.time()

    def target_pose_callback(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        q = msg.pose.orientation
        self.target_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        self.last_target_time = time.time()
        self.reached = False

        self.get_logger().info(
            f"Received new target: "
            f"x={self.target_x:.3f}, y={self.target_y:.3f}, yaw={self.target_yaw:.3f} rad, "
            f"frame_id={msg.header.frame_id}"
        )

    def debug_log(self, text: str):
        if not self.debug_print:
            return

        now = time.time()
        if now - self.last_debug_time < self.debug_period:
            return

        self.last_debug_time = now
        self.get_logger().info(text)

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    def make_gamepad_msg(
        self,
        left_stick_x: float = 0.0,
        left_stick_y: float = 0.0,
        right_stick_y: float = 0.0,
    ) -> GamepadKeys:
        msg = GamepadKeys()
        msg.timestamp = int(time.time() * 1_000_000)

        msg.digital_states = [0] * 12
        msg.analog_states = [0.0] * 6

        # analog_states:
        # 0 LT
        # 1 RT
        # 2 Left Stick X:  前后，正数前进
        # 3 Left Stick Y:  左右，正数向右
        # 4 Right Stick X
        # 5 Right Stick Y: 转向，正数右转
        msg.analog_states[2] = float(left_stick_x)
        msg.analog_states[3] = float(left_stick_y)
        msg.analog_states[5] = float(right_stick_y)

        return msg

    def publish_gamepad(
        self,
        left_stick_x: float = 0.0,
        left_stick_y: float = 0.0,
        right_stick_y: float = 0.0,
    ):
        msg = self.make_gamepad_msg(
            left_stick_x=left_stick_x,
            left_stick_y=left_stick_y,
            right_stick_y=right_stick_y,
        )
        self.lcm_handle.publish(self.lcm_channel, msg.encode())

    def publish_zero(self, reason: str):
        self.publish_gamepad(0.0, 0.0, 0.0)
        self.debug_log(
            f"{reason} | joystick: LX_forward=0.000, LY_right=0.000, RY_turn_right=0.000"
        )

    def control_loop(self):
        now = time.time()

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.publish_zero("No current pose received")
            return

        if self.target_x is None or self.target_y is None or self.target_yaw is None:
            self.publish_zero("No target pose received")
            return

        if now - self.last_target_time > self.target_timeout:
            self.publish_zero(
                f"Target timeout: last target {now - self.last_target_time:.2f}s ago"
            )
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)

        yaw = self.current_yaw
        yaw_error = normalize_angle(self.target_yaw - yaw)

        # 世界坐标误差转到机器人机体系。
        # body_forward > 0：目标在机器人前方
        # body_left > 0：目标在机器人左侧
        body_forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_left = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # 你的测试结论：Left Stick Y 正数让机器人向右移动
        body_right = -body_left

        if distance < self.position_tolerance:
            forward_cmd = 0.0
            lateral_cmd = 0.0
        else:
            forward_cmd = self.forward_sign * self.kp_forward * body_forward
            lateral_cmd = self.lateral_sign * self.kp_lateral * body_right

        if abs(yaw_error) < self.yaw_tolerance:
            turn_cmd = 0.0
        else:
            turn_cmd = self.yaw_sign * self.kp_yaw * yaw_error

        forward_cmd = clamp(forward_cmd, -self.max_forward, self.max_forward)
        lateral_cmd = clamp(lateral_cmd, -self.max_lateral, self.max_lateral)
        turn_cmd = clamp(turn_cmd, -self.max_turn, self.max_turn)

        forward_cmd = self.apply_deadband(forward_cmd)
        lateral_cmd = self.apply_deadband(lateral_cmd)
        turn_cmd = self.apply_deadband(turn_cmd)

        if distance < self.position_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.publish_gamepad(0.0, 0.0, 0.0)

            if not self.reached:
                self.get_logger().info(
                    f"Target reached. "
                    f"current=({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.3f}), "
                    f"target=({self.target_x:.3f}, {self.target_y:.3f}, {self.target_yaw:.3f}), "
                    f"distance={distance:.3f}, yaw_error={yaw_error:.3f}"
                )
                self.reached = True

            return

        self.reached = False

        self.publish_gamepad(
            left_stick_x=forward_cmd,
            left_stick_y=lateral_cmd,
            right_stick_y=turn_cmd,
        )

        self.debug_log(
            f"target=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_yaw:.2f}) | "
            f"current=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f}) | "
            f"distance={distance:.3f}, yaw_error={yaw_error:.3f} | "
            f"body_error: forward={body_forward:.3f}, right={body_right:.3f} | "
            f"joystick: LX_forward={forward_cmd:.3f}, "
            f"LY_right={lateral_cmd:.3f}, "
            f"RY_turn_right={turn_cmd:.3f}"
        )


def main():
    print("running this 22222 !!!!")
    rclpy.init()
    node = GlobalPoseGamepadController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_gamepad(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print("running this one !!!!")
    main()
