#!/usr/bin/env python3
"""ROS 2 node: execute absolute-position instructions on MAB MD drives.

Runs on the Pi that carries the CANdle. Subscribes to

    /roboarm/joint_command   std_msgs/Float64MultiArray

where data[i] is the ABSOLUTE target position in radians for joint i.
Joints map 1:1 to MD CAN IDs (default [779], the MA-p-45-10_KV75). Each
move is
executed with candletool's trapezoidal position profile; the positions
streamed during the move are published on /joint_states. A new command
preempts the move in flight. One candletool runs at a time (the CANdle is
single-client), so moves are serialized.

Usage:  source /opt/ros/humble/setup.bash && python3 mab_arm_node.py
        ros2 topic pub --once /roboarm/joint_command \
            std_msgs/msg/Float64MultiArray "{data: [1.0]}"
"""
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

POS_RE = re.compile(r'[Pp]osition:\s*(-?\d+\.?\d*)')


class MabArmDriver(Node):

    def __init__(self):
        super().__init__('mab_arm_driver')
        self.declare_parameter('can_ids', [779])
        self.declare_parameter('joint_command_topic', '/roboarm/joint_command')
        self.declare_parameter('poll_period', 2.0)
        can_ids = [int(i) for i in self.get_parameter('can_ids').value]
        self.joints = [(f'joint{i + 1}', cid) for i, cid in enumerate(can_ids)]
        cmd_topic = self.get_parameter('joint_command_topic').value
        self.create_subscription(Float64MultiArray, cmd_topic,
                                 self.on_command, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target = None
        self.lock = threading.Lock()
        self.proc = None
        self.positions = [0.0] * len(self.joints)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(
            f'joints {self.joints}, listening on {cmd_topic}')

    def on_command(self, msg):
        n = len(self.joints)
        if len(msg.data) < n:
            self.get_logger().warn(
                f'command has {len(msg.data)} values, need {n}; ignored')
            return
        with self.lock:
            self.target = [float(v) for v in msg.data[:n]]
        self.get_logger().info(f'absolute target {self.target} rad')
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()

    def publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [name for name, _ in self.joints]
        js.position = list(self.positions)
        self.js_pub.publish(js)

    def move_joint(self, index, can_id, target):
        """Run one absolute move; stream positions; False if preempted."""
        self.proc = subprocess.Popen(
            ['candletool', 'md', '--id', str(can_id),
             'test', 'absolute', f'{target:.4f}'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in self.proc.stdout:
            m = POS_RE.search(line)
            if m:
                self.positions[index] = float(m.group(1))
                self.publish_state()
        self.proc.wait()
        return self.proc.returncode == 0

    def read_positions(self):
        for i, (_, can_id) in enumerate(self.joints):
            out = subprocess.run(
                ['candletool', 'md', '--id', str(can_id), 'info'],
                capture_output=True, text=True, timeout=30).stdout
            m = POS_RE.search(out)
            if m:
                self.positions[i] = float(m.group(1))
        self.publish_state()

    def worker(self):
        poll = self.get_parameter('poll_period').value
        while True:
            with self.lock:
                target = self.target
                self.target = None
            if target is None:
                try:
                    self.read_positions()
                except (subprocess.TimeoutExpired, OSError) as e:
                    self.get_logger().warn(f'poll failed: {e}')
                time.sleep(poll)
                continue
            for i, (_, can_id) in enumerate(self.joints):
                if not self.move_joint(i, can_id, target[i]):
                    self.get_logger().warn(
                        f'joint {i + 1} move interrupted (rc != 0)')
                    break
            else:
                self.get_logger().info('target reached')


def main():
    rclpy.init()
    node = MabArmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.proc and node.proc.poll() is None:
            node.proc.terminate()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
