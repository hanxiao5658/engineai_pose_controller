#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T800 机器人运动状态切换及标准 ROS2 状态发布节点。

建议放置路径：
  ~/ros2_ws/src/engineai_pose_controller/engineai_pose_controller/my_switch_robotState.py

订阅（厂商自定义）：
  /motion/motion_state                 interface_protocol/msg/MotionState
发布（ROS2 标准类型）：
  /my_ros2/motion/motion_state         std_msgs/msg/String（JSON 文本）
  /motion/set_motion_state             interface_protocol/msg/MotionStateRequest
提供（ROS2 标准 Trigger Service）：
  /my_ros2/switch_motion_srv/set_to_walking
  /my_ros2/switch_motion_srv/set_to_lowerBodyBalance

重要：只有在符合指定初始状态时才发送一次切换请求；
      切换成功需要收到 /motion/motion_state 的实际状态反馈确认。
"""

import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from interface_protocol.msg import MotionState, MotionStateRequest
from std_msgs.msg import String
from std_srvs.srv import Trigger


# 只允许以下两种定向切换，不允许客户端任意填写目标运动模式。
WALKING_STATE = "rl_basic"
LOWER_BODY_BALANCE_STATE = "lower_body_balance"

# 与用户约定的状态不匹配提示保持完全一致。
WRONG_STATE_TO_WALKING = (
    "wrong robot state, check the situation and switch to walking manually"
)
WRONG_STATE_TO_BALANCE = (
    "wrong robot state, check the situation and switch to lower body balance manually"
)


class RobotStateSwitcher(Node):
    def __init__(self):
        super().__init__("my_switch_robot_state")

        # 最多等待切换完成的时间（秒），参考官方示例的 3 秒。
        self.declare_parameter("switch_timeout_sec", 3.0)

        # 状态超过该时间未更新，就拒绝发送切换指令，防止使用旧状态。
        self.declare_parameter("state_max_age_sec", 2.0)

        self._switch_timeout = float(
            self.get_parameter("switch_timeout_sec").value
        )
        self._state_max_age = float(
            self.get_parameter("state_max_age_sec").value
        )
        if self._switch_timeout <= 0 or self._state_max_age <= 0:
            raise ValueError("switch_timeout_sec 和 state_max_age_sec 必须大于 0")

        # 一个线程等待实际运动状态变化，另一个线程持续处理订阅回调。
        # 必须配合 MultiThreadedExecutor，防止 Service 回调阻塞订阅回调。
        self._cb_group = ReentrantCallbackGroup()

        # Condition 负责在收到状态反馈时唤醒等待中的 Service。
        self._state_condition = threading.Condition()
        self._current_state = None
        self._available_transitions = []
        self._state_received_at = 0.0  # time.monotonic()，不是 ROS 时间
        self._state_sequence = 0

        # 禁止两个 Service 同时切换运动模式。
        self._switch_lock = threading.Lock()

        # 官方示例对 MotionState 使用 BEST_EFFORT 订阅。
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            MotionState,
            "/motion/motion_state",
            self._motion_state_callback,
            state_qos,
            callback_group=self._cb_group,
        )

        # 官方示例对 MotionStateRequest 使用 RELIABLE 发布。
        request_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._vendor_request_pub = self.create_publisher(
            MotionStateRequest,
            "/motion/set_motion_state",
            request_qos,
        )

        # 使用标准 std_msgs/String 包含 JSON，避免其他容器依赖厂商消息。
        # JSON 字段：current_motion_task、available_transition_motions。
        self._standard_state_pub = self.create_publisher(
            String,
            "/my_ros2/motion/motion_state",
            10,
        )

        self.create_service(
            Trigger,
            "/my_ros2/switch_motion_srv/set_to_walking",
            self._handle_to_walking,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger,
            "/my_ros2/switch_motion_srv/set_to_lowerBodyBalance",
            self._handle_to_lower_body_balance,
            callback_group=self._cb_group,
        )

        self.get_logger().info("Robot state switcher started; waiting for vendor motion state")

    def _motion_state_callback(self, msg):
        """缓存厂商反馈，并原样转为 ROS 标准 String 的 JSON 内容。"""
        state_name = str(msg.current_motion_task)
        transitions = list(msg.available_transition_motions)

        with self._state_condition:
            self._current_state = state_name
            self._available_transitions = transitions
            self._state_received_at = time.monotonic()
            self._state_sequence += 1
            self._state_condition.notify_all()

        # 不使用自定义消息发布，方便 ROS2 其他节点及 ros1_bridge 读取。
        standard_msg = String()
        standard_msg.data = json.dumps(
            {
                "current_motion_task": state_name,
                "available_transition_motions": transitions,
            },
            ensure_ascii=False,
        )
        self._standard_state_pub.publish(standard_msg)

    def _handle_to_walking(self, _request, response):
        """仅允许 lower_body_balance -> rl_basic。"""
        return self._switch_once(
            response=response,
            required_current=LOWER_BODY_BALANCE_STATE,
            target=WALKING_STATE,
            wrong_state_message=WRONG_STATE_TO_WALKING,
        )

    def _handle_to_lower_body_balance(self, _request, response):
        """仅允许 rl_basic -> lower_body_balance。"""
        return self._switch_once(
            response=response,
            required_current=WALKING_STATE,
            target=LOWER_BODY_BALANCE_STATE,
            wrong_state_message=WRONG_STATE_TO_BALANCE,
        )

    def _switch_once(self, response, required_current, target, wrong_state_message):
        """发送一次请求，等待厂商真实状态反馈；不自动重发。"""
        response.success = False

        if not self._switch_lock.acquire(blocking=False):
            response.message = "another motion state switch is in progress"
            return response

        try:
            with self._state_condition:
                now = time.monotonic()

                # 没收到过状态或反馈已过期，拒绝切换。
                if (
                    self._current_state is None
                    or now - self._state_received_at > self._state_max_age
                ):
                    response.message = (
                        "motion state unavailable or stale; "
                        "check /motion/motion_state before switching manually"
                    )
                    self.get_logger().warning(response.message)
                    return response

                # 用户指定的唯一允许起始状态；其余状态严格拒绝。
                if self._current_state != required_current:
                    response.message = wrong_state_message
                    self.get_logger().warning(
                        "Rejected transition %s -> %s: current=%s"
                        % (required_current, target, self._current_state)
                    )
                    return response

                # 此处故意不按 available_transition_motions 额外判断：
                # 按用户要求，仅以 current_motion_task 是否为指定状态作为授权条件。
                # 厂商最终能否完成切换，以实际状态反馈为准。
                start_sequence = self._state_sequence
                deadline = time.monotonic() + self._switch_timeout

                vendor_request = MotionStateRequest()
                vendor_request.target_motion_name = target
                self._vendor_request_pub.publish(vendor_request)
                self.get_logger().info(
                    "Published motion switch request: %s -> %s"
                    % (required_current, target)
                )

                while rclpy.ok():
                    # 必须在本次发布请求之后收到新的厂商状态反馈。
                    if (
                        self._state_sequence > start_sequence
                        and self._current_state == target
                    ):
                        response.success = True
                        response.message = "motion state switched to %s" % target
                        self.get_logger().info(response.message)
                        return response

                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        response.message = (
                            "timeout waiting for %s; current state=%s; "
                            "check robot state before retrying"
                            % (target, self._current_state)
                        )
                        self.get_logger().warning(response.message)
                        return response

                    # wait 会释放 Condition，允许订阅回调更新状态并唤醒。
                    self._state_condition.wait(timeout=remaining)

                response.message = "ROS2 is shutting down; state result is unknown"
                return response

        finally:
            self._switch_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateSwitcher()

    # 多线程执行器很重要：两个 Trigger Service 有同步等待逻辑，
    # /motion/motion_state 仍须被其他线程实时处理。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
