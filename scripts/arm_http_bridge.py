#!/usr/bin/env python3
"""HTTP-to-ROS bridge for the MAB arm node.

Run on the Pi next to mab_arm_node.py (same ROS domain), started by
run_arm.sh. Gives non-ROS callers — the bartender_api on the main machine,
and through it the VLM — a plain HTTP way to command the arm:

    POST /arm/move    {"joints": [0.5]}      absolute radians, joint i -> data[i]
    GET  /arm/state                          latest /joint_states
    GET  /health                              always 200, for liveness

Binds 0.0.0.0:8092 by default. Like bartender_api's own warning: this
moves a robot arm with no authentication — only run it on the isolated
10.42.0.x robot LAN.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

MAX_JOINTS = 6


class ArmBridge(Node):

    def __init__(self):
        super().__init__('arm_http_bridge')
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/roboarm/joint_command', 10)
        self.latest = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states',
                                 self.on_state, 10)
        self.get_logger().info('arm_http_bridge up')

    def on_state(self, msg):
        with self.lock:
            self.latest = msg

    def state(self):
        with self.lock:
            msg = self.latest
        if msg is None:
            return None
        return {'ok': True, 'joints': list(msg.position),
                'names': list(msg.name),
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}


def make_handler(node):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def _send(self, code, payload):
            raw = json.dumps(payload).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            path = self.path.split('?')[0].rstrip('/')
            if path == '/health':
                self._send(200, {'ok': True, 'node': 'arm_http_bridge'})
            elif path == '/arm/state':
                result = node.state()
                if result is None:
                    self._send(503, {'ok': False,
                                     'error': 'no /joint_states yet'})
                else:
                    self._send(200, result)
            else:
                self._send(404, {'ok': False, 'error': 'not found'})

        def do_POST(self):
            path = self.path.split('?')[0].rstrip('/')
            if path != '/arm/move':
                self._send(404, {'ok': False, 'error': 'not found'})
                return
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                self._send(400, {'ok': False, 'error': 'bad request body'})
                return
            joints = body.get('joints') if isinstance(body, dict) else None
            # bools are ints in Python: 'joints': [true] must not move the arm.
            if (not isinstance(joints, list) or not 1 <= len(joints) <= MAX_JOINTS
                    or not all(isinstance(j, (int, float))
                               and not isinstance(j, bool)
                               and j == j and abs(j) != float('inf')
                               for j in joints)):
                self._send(400, {'ok': False,
                                 'error': f'joints must be 1-{MAX_JOINTS} '
                                          'finite numbers (absolute radians)'})
                return
            msg = Float64MultiArray()
            msg.data = [float(j) for j in joints]
            node.cmd_pub.publish(msg)
            node.get_logger().info(f'move command {list(msg.data)}')
            self._send(200, {'ok': True, 'sent': list(msg.data)})

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    rclpy.init()
    node = ArmBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', 8092), make_handler(node))
    print('arm_http_bridge: http://0.0.0.0:8092  (POST /arm/move, '
          'GET /arm/state) -- unauthenticated robot motion, keep this on '
          'the isolated robot LAN')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
