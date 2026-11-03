#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T800 simulation adapter: EngineAI BodyVelCmd -> virtual gamepad LCM.

Input (identical to the vendor's body_velocity_control_example.py):
    /motion/body_vel_cmd  interface_protocol/msg/BodyVelCmd
      header.frame_id = "body"
      linear_velocity  = [vx, vy]  (m/s, body frame, +x forward, +y left)
      yaw_velocity     = wz        (rad/s, +z counterclockwise)

Output (same as the previously working global_pose_gamepad_controller.py):
    LCM channel virtual_gamepad/gamepad_keys; lcm_msgs.data.GamepadKeys

The gamepad API accepts dimensionless stick commands, not physical velocities.
The *_scale parameters are *empirical* stick-per-physical-unit gains; calibrate
these on the simulator before relying on speed accuracy.

No pose feedback or position P-control is used in this adapter.
"""

import math
import os
import sys
import time
from typing import Optional, Tuple

# Preserve the path and LCM channel of the original, tested controller.
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
from interface_protocol.msg import BodyVelCmd
from lcm_msgs.data import GamepadKeys


def clamp(value: float, bound: float) -> float:
    limit = abs(bound)
    return max(-limit, min(limit, value))


class GlobalPoseGamepadController(Node):
    """Keep the old class / console-script name for drop-in compatibility."""

    def __init__(self):
        super().__init__("global_pose_gamepad_controller")

        # Topic and LCM: vendor-facing topic is NOT renamed to /my_ros2/... .
        self.declare_parameter("body_vel_topic", "/motion/body_vel_cmd")
        self.declare_parameter("expected_frame_id", "body")
        self.declare_parameter("lcm_url", "udpm://239.255.76.67:7667?ttl=0")
        self.declare_parameter("lcm_channel", "virtual_gamepad/gamepad_keys")

        # Vendor example publishes at 100 Hz. Forward the latest command at 100 Hz.
        self.declare_parameter("control_rate", 100.0)
        # Invalidate commands on loss of publisher (monotonic/wall time, not /clock).
        self.declare_parameter("cmd_timeout", 0.3)

        # Values convert physical units to dimensionless gamepad axes:
        # 0.5 m/s -> 0.5 on forward stick at the default 1.0 scale.
        # These are initial calibration values, NOT an accurate speed model.
        self.declare_parameter("forward_scale", 1.0)   # stick per (m/s)
        self.declare_parameter("lateral_scale", 1.0)   # stick per (m/s)
        self.declare_parameter("yaw_scale", 1.0)       # stick per (rad/s)

        # Preserve empirically verified original axes and signs.
        # Old LCM: left stick X +forward, left stick Y +right,
        #          right stick Y +right turn.
        # BodyVelCmd (usual ROS body): +x forward, +y left, +yaw left.
        self.declare_parameter("forward_sign", 1.0)
        self.declare_parameter("lateral_sign", -1.0)
        self.declare_parameter("yaw_sign", -1.0)

        self.declare_parameter("max_forward", 0.9)
        self.declare_parameter("max_lateral", 0.2)
        self.declare_parameter("max_turn", 0.9)
        self.declare_parameter("deadband", 0.03)
        self.declare_parameter("debug_print", True)
        self.declare_parameter("debug_period", 1.0)

        def param(name):
            return self.get_parameter(name).value

        self.body_vel_topic = str(param("body_vel_topic"))
        self.expected_frame_id = str(param("expected_frame_id"))
        self.lcm_channel = str(param("lcm_channel"))
        self.control_rate = float(param("control_rate"))
        self.cmd_timeout = float(param("cmd_timeout"))
        self.forward_scale = float(param("forward_scale"))
        self.lateral_scale = float(param("lateral_scale"))
        self.yaw_scale = float(param("yaw_scale"))
        self.forward_sign = float(param("forward_sign"))
        self.lateral_sign = float(param("lateral_sign"))
        self.yaw_sign = float(param("yaw_sign"))
        self.max_forward = min(abs(float(param("max_forward"))), 1.0)
        self.max_lateral = min(abs(float(param("max_lateral"))), 1.0)
        self.max_turn = min(abs(float(param("max_turn"))), 1.0)
        self.deadband = abs(float(param("deadband")))
        self.debug_print = bool(param("debug_print"))
        self.debug_period = float(param("debug_period"))

        if self.control_rate <= 0.0 or self.cmd_timeout <= 0.0:
            raise ValueError("control_rate and cmd_timeout must both be > 0")
        for name in ("forward_scale", "lateral_scale", "yaw_scale"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(name + " must be finite and nonnegative")

        self.last_rx_monotonic: Optional[float] = None
        self.last_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_debug_monotonic = 0.0
        self.last_warning_monotonic = 0.0
        self.reported_timeout = False

        self.lcm_handle = lcm.LCM(str(param("lcm_url")))

        # BEST_EFFORT subscriber is compatible with the vendor's default RELIABLE
        # publisher, as well as with a BEST_EFFORT publisher in some simulators.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            BodyVelCmd, self.body_vel_topic, self.body_vel_callback, qos
        )
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            "T800 BodyVelCmd -> LCM adapter started: "
            f"subscribe={self.body_vel_topic} "
            f"(interface_protocol/msg/BodyVelCmd); "
            f"publish LCM={self.lcm_channel}; "
            f"rate={self.control_rate:g}Hz; timeout={self.cmd_timeout:g}s"
        )
        self.publish_gamepad(0.0, 0.0, 0.0)

    def warning_throttled(self, message: str):
        now = time.monotonic()
        if now - self.last_warning_monotonic >= 2.0:
            self.get_logger().warning(message)
            self.last_warning_monotonic = now

    def body_vel_callback(self, msg: BodyVelCmd):
        frame = str(msg.header.frame_id)
        if self.expected_frame_id and frame != self.expected_frame_id:
            self.last_rx_monotonic = None
            self.last_cmd = (0.0, 0.0, 0.0)
            self.warning_throttled(
                f"Rejected command frame_id={frame!r}; expected {self.expected_frame_id!r}"
            )
            return

        if len(msg.linear_velocity) != 2:
            self.last_rx_monotonic = None
            self.last_cmd = (0.0, 0.0, 0.0)
            self.warning_throttled("Rejected command: linear_velocity must have 2 entries")
            return

        vx = float(msg.linear_velocity[0])
        vy = float(msg.linear_velocity[1])
        wz = float(msg.yaw_velocity)
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.last_rx_monotonic = None
            self.last_cmd = (0.0, 0.0, 0.0)
            self.warning_throttled("Rejected command: non-finite velocity")
            return

        self.last_cmd = (vx, vy, wz)
        self.last_rx_monotonic = time.monotonic()
        self.reported_timeout = False

    def deadband_axis(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadband else value

    def convert_to_sticks(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        return (
            self.deadband_axis(clamp(self.forward_sign * self.forward_scale * vx, self.max_forward)),
            self.deadband_axis(clamp(self.lateral_sign * self.lateral_scale * vy, self.max_lateral)),
            self.deadband_axis(clamp(self.yaw_sign * self.yaw_scale * wz, self.max_turn)),
        )

    def publish_gamepad(self, left_stick_x: float, left_stick_y: float,
                        right_stick_y: float):
        # Preserve the original LCM GamepadKeys byte schema exactly.
        msg = GamepadKeys()
        msg.timestamp = int(time.time() * 1_000_000)
        msg.digital_states = [0] * 12
        msg.analog_states = [0.0] * 6
        msg.analog_states[2] = float(left_stick_x)
        msg.analog_states[3] = float(left_stick_y)
        msg.analog_states[5] = float(right_stick_y)
        self.lcm_handle.publish(self.lcm_channel, msg.encode())

    def control_loop(self):
        now = time.monotonic()
        fresh = (
            self.last_rx_monotonic is not None
            #and now - self.last_rx_monotonic <= self.cmd_timeout
        )
        if fresh:
            vx, vy, wz = self.last_cmd
            sticks = self.convert_to_sticks(vx, vy, wz)
        else:
            sticks = (0.0, 0.0, 0.0)
            if self.last_rx_monotonic is not None and not self.reported_timeout:
                self.get_logger().warning(
                    f"No new BodyVelCmd for {self.cmd_timeout:.2f}s: publishing neutral sticks"
                )
                self.reported_timeout = True

        self.publish_gamepad(*sticks)
        if self.debug_print and now - self.last_debug_monotonic >= self.debug_period:
            self.last_debug_monotonic = now
            if fresh:
                self.get_logger().info(
                    "BodyVelCmd body(m/s,rad/s): "
                    f"vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} -> "
                    f"LCM sticks: LX={sticks[0]:+.3f} LY={sticks[1]:+.3f} "
                    f"RY={sticks[2]:+.3f}"
                )
            else:
                self.get_logger().info("Waiting for BodyVelCmd; sending neutral sticks")

    def stop(self):
        # Best-effort neutral command at program exit; not a physical E-stop.
        for _ in range(3):
            try:
                self.publish_gamepad(0.0, 0.0, 0.0)
            except Exception:
                break
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GlobalPoseGamepadController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
