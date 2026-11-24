#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T800 机体速度适配器：保留安全门控，但不检查速度数值合法性。

输入（Docker3 / ROS2 Humble）：
  /my_ros1/motion/body_vel_cmd              geometry_msgs/msg/Twist
  /my_ros1/navigation/controller_heartbeat std_msgs/msg/String
  /hardware/gamepad_keys                   interface_protocol/msg/GamepadKeys

正式目标输出：
  /motion/body_vel_cmd                     interface_protocol/msg/BodyVelCmd

安全策略：
  * 初始 DISARMED。
  * heartbeat、Twist、gamepad 都有超时保护。
  * 必须在输入健康且摇杆居中时连续单击 Y 三次才能 ARMED。
  * 连续单击 A 三次进入 DISARMED。
  * 手柄断流/断开、heartbeat 丢失、motion_allowed=0、Twist 过期、
    或摇杆被人为拨动时，立即撤销授权并输出零速度。
  * 撤销授权后不会自动恢复 ARMED，必须再次三击 Y。
  * ARMED 后必须收到新的 Twist，避免恢复 ARM 前缓存的旧速度。

特别说明：
  * 本脚本【不再检查 Twist 速度内容是否合法】。
  * 不做 max_vx / max_vy / max_wz 限幅。
  * 不检查 linear.z / angular.x / angular.y 是否为 0。
  * 不检查 NaN / Inf 等数值合法性。
  * Twist.linear.x、linear.y、angular.z 会直接用于生成 BodyVelCmd。
  * 速度是否合法、速度上限等责任由上游控制器承担。

⚠️ 当前真实机器人 Publisher 与 publish() 调用均保持“代码已写好但注释掉”的状态。
   文件内已明确标出真正向机器人发布速度的那一行。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from interface_protocol.msg import BodyVelCmd, GamepadKeys
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import String


class SafetyGate:
    """与 ROS2 框架分离的门控逻辑，统一使用 monotonic 计时（与 ROS /clock 无关）。"""

    def __init__(self, *, heartbeat_timeout=0.5, cmd_timeout=0.5,
                 gamepad_timeout=0.5, stick_threshold=0.3,
                 stick_indices=(2, 3, 4, 5), triple_window=2.5,
                 key_debounce=0.12):
        if min(heartbeat_timeout, cmd_timeout, gamepad_timeout,
               triple_window) <= 0:
            raise ValueError('所有 timeout 和 triple_window 都必须 > 0')
        if not (0.0 < stick_threshold <= 1.0):
            raise ValueError('stick_threshold 应在 (0, 1]')
        if not (0 <= key_debounce < triple_window):
            raise ValueError('key_debounce 应在 [0, triple_window)')
        if not stick_indices or any(i not in range(6) for i in stick_indices):
            raise ValueError('stick_indices 必须来自 0..5')

        self.heartbeat_timeout = heartbeat_timeout
        self.cmd_timeout = cmd_timeout
        self.gamepad_timeout = gamepad_timeout
        self.stick_threshold = stick_threshold
        self.stick_indices = tuple(stick_indices)
        self.triple_window = triple_window
        self.key_debounce = key_debounce
        self.armed = False
        self.armed_since = None
        self.reason = 'STARTUP_DISARMED'
        self.last_disarm_reason = 'STARTUP_DISARMED'
        self.last_hb_time = None
        self.last_hb_seq = None
        self.motion_allowed = False
        self.last_cmd_time = None
        self.cmd = None
        self.last_pad_time = None
        self.pad_connected = False
        self.sticks_neutral = False
        self.have_pad_baseline = False
        self.prev_y = False
        self.prev_a = False
        self.last_edge_time = {'Y': None, 'A': None}
        self.clicks = {'Y': 0, 'A': 0}
        self.first_click_time = {'Y': None, 'A': None}

    def reset_clicks(self):
        for key in ('Y', 'A'):
            self.clicks[key] = 0
            self.first_click_time[key] = None

    def disarm(self, reason):
        was_armed = self.armed
        self.armed = False
        self.armed_since = None
        self.reason = reason
        self.last_disarm_reason = reason
        self.reset_clicks()
        return was_armed

    def on_heartbeat(self, data, now):
        """上游格式：seq=...|state=...|enabled=...|motion_allowed=...|..."""
        try:
            fields = dict(p.split('=', 1) for p in data.split('|'))
            seq = int(fields['seq'])
            if seq < 0 or fields['motion_allowed'] not in ('0', '1') \
                    or fields['enabled'] not in ('0', '1') or not fields['state']:
                raise ValueError('invalid heartbeat values')
        except (ValueError, KeyError):
            self.motion_allowed = False
            self.disarm('INVALID_HEARTBEAT')
            return

        if self.last_hb_seq is not None:
            if seq == self.last_hb_seq:
                # 不允许重复 seq 刷新心跳新鲜度（防止桥接重播同一条消息）。
                return
            if seq < self.last_hb_seq:
                self.disarm('HEARTBEAT_SEQ_RESTART')
        self.last_hb_seq = seq
        self.last_hb_time = now
        self.motion_allowed = fields['motion_allowed'] == '1' and fields['enabled'] == '1'
        if not self.motion_allowed:
            self.disarm('NAVIGATION_NOT_ALLOWED')

    def on_twist(self, vx, vy, wz, other, now):
        """记录最新 Twist。

        注意：这里不对速度内容做任何合法性检查或限幅。
        只记录 linear.x、linear.y、angular.z，并刷新 Twist 新鲜度时间。
        linear.z、angular.x、angular.y 不参与 BodyVelCmd 转换，直接忽略。
        """
        self.cmd = (float(vx), float(vy), float(wz))
        self.last_cmd_time = now


    def on_gamepad(self, digital, analog, hardware_connected, now,
                   y_index=5, a_index=2):
        if len(digital) < 12 or len(analog) < 6 or not all(
                math.isfinite(float(v)) and abs(float(v)) <= 1.05 for v in analog):
            self.pad_connected = False
            self.disarm('INVALID_GAMEPAD')
            return

        self.last_pad_time = now
        self.pad_connected = bool(hardware_connected)
        self.sticks_neutral = all(abs(analog[i]) <= self.stick_threshold
                                  for i in self.stick_indices)

        y_down = bool(digital[y_index])
        a_down = bool(digital[a_index])
        if not self.have_pad_baseline:
            # 第一帧只建立上次按键状态，避免启动时按着 Y 被算作一次按下。
            self.have_pad_baseline = True
            self.prev_y, self.prev_a = y_down, a_down
            return

        y_edge = y_down and not self.prev_y
        a_edge = a_down and not self.prev_a
        self.prev_y, self.prev_a = y_down, a_down

        if not self.pad_connected:
            self.disarm('HARDWARE_GAMEPAD_DISCONNECTED')
            return
        if not self.sticks_neutral:
            self.disarm('JOYSTICK_TAKEOVER')
            return
        if y_down and a_down:
            self.disarm('SIMULTANEOUS_Y_A')
            return

        if a_edge:
            self._click('A', now)
            if self.clicks['A'] == 3:
                self.disarm('TRIPLE_A_STOP')
        elif y_edge:
            # 正在 ARMED 时，按 Y 不改变授权；防止无意累加或自动切换。
            if not self.armed:
                self._click('Y', now)
                if self.clicks['Y'] == 3:
                    ok, reason = self._input_health(now, require_cmd=True)
                    if ok:
                        self.armed = True
                        self.armed_since = now
                        self.reason = 'TRIPLE_Y_ARMED'
                        self.reset_clicks()
                    else:
                        self.disarm('ARM_REJECTED_' + reason)

    def _click(self, key, now):
        other = 'A' if key == 'Y' else 'Y'
        self.clicks[other] = 0
        self.first_click_time[other] = None
        previous = self.last_edge_time[key]
        if previous is not None and now - previous < self.key_debounce:
            return
        self.last_edge_time[key] = now
        start = self.first_click_time[key]
        if start is None or now - start > self.triple_window:
            self.first_click_time[key] = now
            self.clicks[key] = 1
        else:
            self.clicks[key] += 1
        self.reason = '{}_PRESS_{}/3'.format(key, self.clicks[key])

    def _input_health(self, now, *, require_cmd=True):
        if self.last_pad_time is None or now - self.last_pad_time > self.gamepad_timeout:
            return False, 'GAMEPAD_TIMEOUT'
        if not self.pad_connected:
            return False, 'HARDWARE_GAMEPAD_DISCONNECTED'
        if not self.sticks_neutral:
            return False, 'JOYSTICK_TAKEOVER'
        if self.last_hb_time is None or now - self.last_hb_time > self.heartbeat_timeout:
            return False, 'HEARTBEAT_TIMEOUT'
        if not self.motion_allowed:
            return False, 'NAVIGATION_NOT_ALLOWED'
        if require_cmd:
            # Twist 安全兜底只检查“是否收到”和“是否过期”。
            # 不检查速度数值本身是否合法。
            if (self.cmd is None or self.last_cmd_time is None
                    or now - self.last_cmd_time > self.cmd_timeout):
                return False, 'TWIST_TIMEOUT'

        return True, 'OK'

    def output(self, now):
        """计算当前应输出的速度与实时状态原因。

        注意：
        * 当前输入不健康：立即 DISARM，并输出零速度。
        * 输入恢复健康后不会自动重新 ARMED。
        * DISARMED 且输入健康时返回 DISARMED_INPUTS_OK，
          避免把启动阶段曾出现过的 TWIST_TIMEOUT 等旧原因一直显示下去。
        """
        ok, health_reason = self._input_health(now)

        if not ok:
            self.disarm(health_reason)
            return (0.0, 0.0, 0.0), health_reason

        if not self.armed:
            return (0.0, 0.0, 0.0), 'DISARMED_INPUTS_OK'

        if self.armed_since is None or self.last_cmd_time is None                 or self.last_cmd_time <= self.armed_since:
            # 防止解锁时立即重放已经缓存的旧 Twist。
            return (0.0, 0.0, 0.0), 'WAIT_NEW_TWIST_AFTER_ARM'

        return self.cmd, 'ARMED'


class BodyVelTwistToVendor(Node):
    def __init__(self):
        super().__init__('body_vel_twist_to_vendor')

        self.declare_parameter('input_topic', '/my_ros1/motion/body_vel_cmd')
        self.declare_parameter('heartbeat_topic', '/my_ros1/navigation/controller_heartbeat')
        self.declare_parameter('gamepad_topic', '/hardware/gamepad_keys')
        self.declare_parameter('output_topic', '/motion/body_vel_cmd')
        self.declare_parameter('heartbeat_timeout', 0.5)
        self.declare_parameter('twist_timeout', 0.5)
        self.declare_parameter('gamepad_timeout', 0.5)
        self.declare_parameter('stick_threshold', 0.3)
        self.declare_parameter('stick_indices', [2, 3, 4, 5])
        self.declare_parameter('triple_press_window', 2.5)
        self.declare_parameter('key_debounce', 0.12)
        self.declare_parameter('output_rate_hz', 10.0)
        p = lambda name: self.get_parameter(name).value
        rate = float(p('output_rate_hz'))
        if rate <= 0:
            raise ValueError('output_rate_hz must be > 0')

        self.gate = SafetyGate(
            heartbeat_timeout=float(p('heartbeat_timeout')),
            cmd_timeout=float(p('twist_timeout')),
            gamepad_timeout=float(p('gamepad_timeout')),
            stick_threshold=float(p('stick_threshold')),
            stick_indices=tuple(int(v) for v in p('stick_indices')),
            triple_window=float(p('triple_press_window')),
            key_debounce=float(p('key_debounce')),
        )
        self.output_topic = str(p('output_topic'))
        self.last_logged_reason = None
        self.last_logged_arm_state = None

        # 记录“原始收到”的 Twist，便于调试。
        # 注意：这与 SafetyGate 内部经过合法性检查后的 self.gate.cmd 不同。
        self.last_rx_twist = None
        self.last_rx_twist_time = None

        # BEST_EFFORT 可接收 RELIABLE / BEST_EFFORT 的发布者，volatile 避免旧消息锁存。
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.twist_sub = self.create_subscription(
            Twist, str(p('input_topic')), self._on_twist, qos)
        self.hb_sub = self.create_subscription(
            String, str(p('heartbeat_topic')), self._on_heartbeat, qos)
        self.pad_sub = self.create_subscription(
            GamepadKeys, str(p('gamepad_topic')), self._on_gamepad, qos)

        # ============================================================
        # 真实机器人 Publisher：当前明确禁用
        #
        # ⚠️【真实发布相关代码 1/2】
        # 下面这一行已经写好，但当前保持注释状态，因此本节点不会创建
        # /motion/body_vel_cmd 的真实 Publisher。
        #
        # 若未来完成全部安全测试并确认厂商接口无误，才可以手动解除下一行注释：
        #
        # self.vendor_publisher = self.create_publisher(
        #     BodyVelCmd, self.output_topic, 10)
        #
        # ============================================================

        self.timer = self.create_timer(1.0 / rate, self._tick)

        print('[DRY_RUN] 仅打印，不发布 {}。初始 DISARMED。'.format(
            self.output_topic), flush=True)
        print('[CONFIG] Y={} A={} stick_indices={} threshold={} '
              'heartbeat={}s twist={}s gamepad={}s output_rate={}Hz'.format(
                  GamepadKeys.Y, GamepadKeys.A,
                  self.gate.stick_indices, self.gate.stick_threshold,
                  self.gate.heartbeat_timeout, self.gate.cmd_timeout,
                  self.gate.gamepad_timeout, rate),
              flush=True)

    def _on_twist(self, msg):
        now = time.monotonic()

        # 保存原始接收值，用于日志显示。
        self.last_rx_twist = (
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        )
        self.last_rx_twist_time = now

        self.gate.on_twist(
            msg.linear.x, msg.linear.y, msg.angular.z,
            (msg.linear.z, msg.angular.x, msg.angular.y), now)

    def _on_heartbeat(self, msg):
        self.gate.on_heartbeat(msg.data, time.monotonic())

    def _on_gamepad(self, msg):
        self.gate.on_gamepad(
            msg.digital_states, msg.analog_states,
            msg.hardware_connected, time.monotonic(),
            y_index=GamepadKeys.Y, a_index=GamepadKeys.A)

    def _tick(self):
        now = time.monotonic()
        (vx, vy, wz), reason = self.gate.output(now)

        out = BodyVelCmd()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'body'
        out.linear_velocity = [vx, vy]
        out.yaw_velocity = wz

        # ============================================================
        # 真实机器人速度发布：当前明确禁用
        #
        # ⚠️【真实发布相关代码 2/2 —— 最关键的一行】
        # 下一行才是真正把经过安全门控后的速度发送到厂商机器人的代码。
        # 当前保持注释状态，因此本脚本无论 ARMED / DISARMED 都不会真正控制机器人。
        #
        # ↓↓↓【真正向机器人发送速度的关键一行】当前保持注释 ↓↓↓
        # self.vendor_publisher.publish(out)
        # ↑↑↑ 只有解除这一行注释后，速度才会真正发送给机器人 ↑↑↑
        #
        # ⚠️ 只有在同时解除上方 Publisher 创建代码的注释，并完成全部安全验证后，
        # 才可以解除这一行注释。
        # ============================================================

        # ------------------------------------------------------------
        # 当前 ARM / DISARM 状态
        # ------------------------------------------------------------
        arm_state = 'ARMED' if self.gate.armed else 'DISARMED'

        if (reason != self.last_logged_reason
                or arm_state != self.last_logged_arm_state):
            print(
                '[STATE] {} | reason={} | last_disarm={}'.format(
                    arm_state, reason, self.gate.last_disarm_reason
                ),
                flush=True
            )
            self.last_logged_reason = reason
            self.last_logged_arm_state = arm_state

        # ------------------------------------------------------------
        # RX：最近一次收到的 /my_ros1/motion/body_vel_cmd
        # ------------------------------------------------------------
        if self.last_rx_twist is None or self.last_rx_twist_time is None:
            rx_text = 'RX Twist: NONE'
            twist_age_text = 'N/A'
        else:
            rx_vx, rx_vy, rx_wz = self.last_rx_twist
            twist_age = now - self.last_rx_twist_time
            rx_text = (
                'RX Twist: vx={:.3f} vy={:.3f} wz={:.3f}'.format(
                    rx_vx, rx_vy, rx_wz
                )
            )
            twist_age_text = '{:.3f}s'.format(twist_age)

        # ------------------------------------------------------------
        # TX：经过安全门控后的最终输出
        # ------------------------------------------------------------
        tx_mode = 'DRY_RUN'
        tx_text = (
            'TX {}: vx={:.3f} vy={:.3f} wz={:.3f}'.format(
                self.output_topic, vx, vy, wz
            )
        )

        print(
            '[{}] {} | reason={} | {} | twist_age={} | {}'.format(
                tx_mode,
                arm_state,
                reason,
                rx_text,
                twist_age_text,
                tx_text,
            ),
            flush=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BodyVelTwistToVendor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


