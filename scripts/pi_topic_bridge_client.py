#!/usr/bin/env python3
"""TCP bridge client: republish the Pi's camera topics on this machine.

Run in the bartender-vla container (or any ROS 2 machine) while
`topic_bridge_server.py` runs on the Pi:

    python3 pi_topic_bridge_client.py                       # defaults
    python3 pi_topic_bridge_client.py --pi 10.42.0.200:7788 --http 8090

Reconnects automatically, republishes each forwarded topic locally as
`sensor_msgs/CompressedImage` under its original name (best-effort QoS),
and with --http serves an MJPEG gallery of the JPEG topics at
http://localhost:PORT/ for quick viewing in a browser.

Protocol: see topic_bridge_server.py on the Pi.
"""
import argparse
import faulthandler
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('server closed')
        buf += chunk
    return buf


class Latest:
    """Latest JPEG per topic, for the HTTP gallery."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {}     # topic -> jpeg bytes

    def put(self, topic, data):
        with self.lock:
            self.frames[topic] = data

    def get(self, topic):
        with self.lock:
            return self.frames.get(topic)


class Client(Node):

    def __init__(self, pi_addr, topics, latest):
        super().__init__('pi_topic_bridge_client')
        self.pi_addr = pi_addr
        self.topics = topics
        self.latest = latest
        self.pubs = {}

    def publish(self, header, data):
        topic = header['topic']
        if topic not in self.pubs:
            self.pubs[topic] = self.create_publisher(
                CompressedImage, topic, qos_profile_sensor_data)
            self.get_logger().info(f'republishing {topic}')
        msg = CompressedImage()
        msg.header.stamp.sec = header['stamp_sec']
        msg.header.stamp.nanosec = header['stamp_nsec']
        msg.header.frame_id = header['frame_id']
        msg.format = header['format']
        msg.data = data.tobytes() if isinstance(data, memoryview) else data
        self.pubs[topic].publish(msg)
        if header['format'].startswith('jpeg'):
            self.latest.put(topic, bytes(msg.data))

    def run(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'connecting to {self.pi_addr}...')
                sock = socket.create_connection(self.pi_addr, timeout=5)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(None)
                sub = '* *' if not self.topics else \
                    '\n'.join(f'SUB {t}' for t in self.topics)
                sock.sendall((sub + '\n').encode())
                self.reader(sock)
            except (OSError, ConnectionError) as e:
                self.get_logger().warn(f'bridge down ({e}); retrying in 2s')
                time.sleep(2)

    def reader(self, sock):
        while True:
            hlen = struct.unpack('>I', recv_exact(sock, 4))[0]
            header = json.loads(recv_exact(sock, hlen))
            dlen = struct.unpack('>I', recv_exact(sock, 4))[0]
            data = recv_exact(sock, dlen)
            self.publish(header, data)


def make_http_handler(latest):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                with latest.lock:
                    topics = sorted(latest.frames)
                parts = ['<html><head><title>Pi cameras</title></head><body>']
                for t in topics:
                    parts.append(
                        f'<div><h3>{t}</h3>'
                        f'<img src="/stream?t={t}" width="640"></div>')
                parts.append('</body></html>')
                body = ''.join(parts).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith('/stream?t='):
                topic = self.path[len('/stream?t='):]
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; '
                                 'boundary=frame')
                self.end_headers()
                while True:
                    frame = latest.get(topic)
                    if frame:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: ' +
                                         str(len(frame)).encode() + b'\r\n\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    time.sleep(1 / 30)
            elif self.path.startswith('/frame?t='):
                topic = self.path[len('/frame?t='):]
                frame = latest.get(topic)
                if frame is None:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_error(404)

        def log_message(self, *a):
            pass
    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pi', default='10.42.0.200:7788',
                   help='host:port of topic_bridge_server.py on the Pi')
    p.add_argument('--topics', nargs='+', default=[
        '/cam0/image_raw/compressed',
        '/cam1/image_raw/compressed',
        '/cam2/d405/color/image_raw/compressed',
    ])
    p.add_argument('--http', type=int, default=8090,
                   help='MJPEG gallery port; 0 disables')
    args = p.parse_args()

    faulthandler.dump_traceback_later(45, repeat=True)
    host, port = args.pi.split(':')
    latest = Latest()
    rclpy.init()
    node = Client((host, int(port)), args.topics, latest)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    if args.http:
        httpd = ThreadingHTTPServer(('0.0.0.0', args.http),
                                    make_http_handler(latest))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        print(f'MJPEG gallery at http://localhost:{args.http}/')
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
