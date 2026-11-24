#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T800 ROS2 GamepadKeys 测试发布器（仅模拟 A/Y + 鼠标滚轮）。

发布唯一话题:
    /hardware/gamepad_keys    interface_protocol/msg/GamepadKeys

窗口内操作:
    A 按下 / 松开 -> digital_states[2] = 1 / 0
    Y 按下 / 松开 -> digital_states[5] = 1 / 0
    鼠标滚轮向上   -> analog_states[2] = +wheel_value（短脉冲）
    鼠标滚轮向下   -> analog_states[2] = -wheel_value（短脉冲）
    脉冲结束       -> analog_states[2] = 0.0

发布期间 hardware_connected=True；其余按键/模拟量始终为 0。
退出时额外发布一次 hardware_connected=False，以提示安全门控失联。

不会发布 Twist、导航 heartbeat 或厂商速度消息。
依赖：ROS2 Humble + interface_protocol；GUI 使用 Python3 tkinter（apt install python3-tk）。
注意：请在无人身危险的隔离测试环境使用，不要与真实手柄发布者同时占用该话题。
"""

import argparse
import time
import tkinter as tk

import rclpy
from interface_protocol.msg import GamepadKeys
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


TOPIC = '/hardware/gamepad_keys'
A_INDEX = 2
Y_INDEX = 5
LEFT_STICK_X_INDEX = 2


class SimulatedGamepadState:
    """将 GUI 事件转换为固定长度的厂商手柄数组，不依赖 ROS。"""

    def __init__(self, wheel_value=0.8, wheel_hold=0.40):
        if not 0.3 < wheel_value <= 1.0:
            raise ValueError('wheel_value 必须 > 0.3 且 <= 1.0，以触发摇杆接管测试')
        if wheel_hold <= 0:
            raise ValueError('wheel_hold 必须 > 0')
        self.wheel_value = float(wheel_value)
        self.wheel_hold = float(wheel_hold)
        self.keys = {'a': False, 'y': False}
        self.wheel_x = 0.0
        self.wheel_deadline = 0.0

    def press(self, key):
        if key in self.keys:
            self.keys[key] = True

    def release(self, key):
        if key in self.keys:
            self.keys[key] = False

    def release_all(self):
        for key in self.keys:
            self.keys[key] = False

    def wheel(self, direction, now):
        self.wheel_x = self.wheel_value if direction > 0 else -self.wheel_value
        self.wheel_deadline = now + self.wheel_hold

    def snapshot(self, now):
        if self.wheel_x != 0.0 and now >= self.wheel_deadline:
            self.wheel_x = 0.0
        digital = [0] * 12
        digital[A_INDEX] = int(self.keys['a'])
        digital[Y_INDEX] = int(self.keys['y'])
        analog = [0.0] * 6
        analog[LEFT_STICK_X_INDEX] = self.wheel_x
        return digital, analog


class TestGamepadPublisher(Node):
    def __init__(self):
        super().__init__('t800_test_keyboard_gamepad')
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(GamepadKeys, TOPIC, qos)

    def publish_state(self, state, connected=True):
        digital, analog = state.snapshot(time.monotonic())
        msg = GamepadKeys()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'virtual_keyboard_gamepad_test'
        msg.hardware_connected = connected
        msg.digital_states = digital
        msg.analog_states = analog
        self.publisher.publish(msg)


class GamepadWindow:
    def __init__(self, node, rate, wheel_value, wheel_hold):
        self.node = node
        self.period_ms = max(10, round(1000.0 / rate))
        self.state = SimulatedGamepadState(wheel_value, wheel_hold)
        self.running = True
        self.pending_release = {}
        self.last_wheel = 0.0

        self.root = tk.Tk()
        self.root.title('T800 /hardware/gamepad_keys 测试器')
        self.root.geometry('610x285')
        self.root.minsize(540, 250)

        tk.Label(self.root, text='T800 手柄测试发布器', font=('Sans', 16, 'bold')).pack(pady=(17, 8))
        tk.Label(self.root, text='仅发布 /hardware/gamepad_keys，hardware_connected = True',
                 font=('Sans', 11)).pack(pady=3)
        tk.Label(self.root, text='在本窗口按 A / Y：模拟手柄 A / Y 按下和松开',
                 font=('Sans', 11)).pack(pady=3)
        tk.Label(self.root, text='鼠标滚轮 ↑ / ↓：模拟 analog_states[2] = ±{:.1f}（维持 {:.2f}s）'.format(
                 wheel_value, wheel_hold), font=('Sans', 11)).pack(pady=3)
        tk.Label(self.root, text='三击 Y 授权、三击 A 停止；滚轮触发摇杆接管',
                 font=('Sans', 11)).pack(pady=3)
        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, font=('Monospace', 12)).pack(pady=12)
        tk.Label(self.root, text='点击窗口获取键盘焦点；关闭窗口或 Ctrl+C 退出',
                 font=('Sans', 10)).pack(pady=(2, 8))

        self.root.bind('<KeyPress>', self._on_key_press)
        self.root.bind('<KeyRelease>', self._on_key_release)
        # Linux / Jetson X11：滚轮是 Button-4 / Button-5。
        self.root.bind('<Button-4>', lambda event: self._on_wheel(+1))
        self.root.bind('<Button-5>', lambda event: self._on_wheel(-1))
        # 兼容 Windows/macOS 的 MouseWheel 事件。
        self.root.bind('<MouseWheel>', lambda event: self._on_wheel(+1 if event.delta > 0 else -1))
        self.root.bind('<FocusOut>', self._on_focus_out)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(200, self.root.focus_force)
        self._update_status()
        self.root.after(self.period_ms, self._tick)

    @staticmethod
    def _key(event):
        return event.keysym.lower()

    def _on_key_press(self, event):
        key = self._key(event)
        if key not in ('a', 'y'):
            return
        # X11 长按自动重复可能产生 KeyRelease -> KeyPress 对。
        # 如果短时间内又来了 KeyPress，取消待执行释放，不会误算多次按键。
        pending = self.pending_release.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        if not self.state.keys[key]:
            self.state.press(key)
            self.node.publish_state(self.state)  # 立即发送按下边沿
            self._update_status()
            print('[KEY] {} DOWN'.format(key.upper()), flush=True)

    def _on_key_release(self, event):
        key = self._key(event)
        if key not in ('a', 'y'):
            return
        pending = self.pending_release.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.pending_release[key] = self.root.after(40, lambda key=key: self._release(key))

    def _release(self, key):
        self.pending_release.pop(key, None)
        if self.state.keys[key]:
            self.state.release(key)
            self.node.publish_state(self.state)  # 立即发送松开边沿
            self._update_status()
            print('[KEY] {} UP'.format(key.upper()), flush=True)

    def _on_focus_out(self, _event):
        # GUI 失去焦点后不应保持“按键一直按下”的状态。
        for pending in self.pending_release.values():
            self.root.after_cancel(pending)
        self.pending_release.clear()
        self.state.release_all()
        self.node.publish_state(self.state)
        self._update_status()

    def _on_wheel(self, direction):
        self.state.wheel(direction, time.monotonic())
        self.last_wheel = self.state.wheel_x
        self.node.publish_state(self.state)  # 立即发送超阈值摇杆脉冲
        self._update_status()
        print('[WHEEL] analog_states[2] = {:+.2f}'.format(self.last_wheel), flush=True)

    def _update_status(self):
        digital, analog = self.state.snapshot(time.monotonic())
        self.status.set('A={}     Y={}     analog[2]={:+.2f}     connected=True'.format(
            digital[A_INDEX], digital[Y_INDEX], analog[LEFT_STICK_X_INDEX]))

    def _tick(self):
        if not self.running:
            return
        previous = self.state.wheel_x
        self.node.publish_state(self.state)
        if previous != self.state.wheel_x:
            print('[WHEEL] analog_states[2] = 0.00 (auto neutral)', flush=True)
        self._update_status()
        self.root.after(self.period_ms, self._tick)

    def close(self):
        if not self.running:
            return
        self.running = False
        self.state.release_all()
        self.state.wheel_x = 0.0
        # 测试器退出时发送一次断开消息；安全门控还会由消息超时自动撤销授权。
        self.node.publish_state(self.state, connected=False)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.node.publish_state(self.state)  # 首帧：建立按键释放状态基线
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description='T800 keyboard+wheel GamepadKeys simulator')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='GamepadKeys 发布频率，默认 20 Hz')
    parser.add_argument('--wheel-value', type=float, default=0.8,
                        help='滚轮触发的 analog_states[2] 数值，默认 0.8；必须 >0.3')
    parser.add_argument('--wheel-hold', type=float, default=0.40,
                        help='滚轮非零脉冲持续秒数，默认 0.40 秒')
    args, ros_args = parser.parse_known_args()
    if args.rate <= 0:
        parser.error('--rate 必须 > 0')

    rclpy.init(args=ros_args)
    node = None
    window = None
    try:
        node = TestGamepadPublisher()
        window = GamepadWindow(node, args.rate, args.wheel_value, args.wheel_hold)
        print('[TEST] Only publishing {}. Domain={}'.format(
            TOPIC, __import__('os').environ.get('ROS_DOMAIN_ID', 'default')), flush=True)
        print('[TEST] Click GUI, press/release A or Y; scroll to simulate stick takeover.', flush=True)
        window.run()
    except KeyboardInterrupt:
        pass
    finally:
        if window is not None and window.running:
            window.close()
        elif window is None and node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
