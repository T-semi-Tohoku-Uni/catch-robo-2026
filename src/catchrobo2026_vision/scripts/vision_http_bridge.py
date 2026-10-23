#!/usr/bin/env python3
"""Serve matching recognition images/results without a robot command interface."""
import collections
import json
import math
import socket
import select
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class LatestFrame:
    def __init__(self, stale_seconds=1.0):
        self.condition = threading.Condition()
        self.stale_seconds = stale_seconds
        self.frame = None
        self.sequence = 0
        self.closed = False
        self.diagnostics = {}

    def publish(self, jpeg, metadata):
        with self.condition:
            if self.closed:
                return
            self.sequence += 1
            self.frame = (self.sequence, jpeg, metadata, time.monotonic())
            self.condition.notify_all()

    def status(self):
        with self.condition:
            frame = self.frame
        if frame is None:
            return dict(available=False, stale=True, age_ms=None, source_age_ms=None,
                        sequence=0, stamp=None, frame_id='', width=0, height=0,
                        detected=None, center_pixel=None, jpeg_bytes=0)
        seq, jpeg, metadata, received = frame
        age = max(0.0, time.monotonic() - received)
        source_age = max(0.0, metadata['source_age_ms'] + age * 1000)
        stale = max(age * 1000, source_age) > self.stale_seconds * 1000
        result = dict(metadata, available=True, stale=stale, age_ms=age * 1000,
                      source_age_ms=source_age, sequence=seq, jpeg_bytes=len(jpeg))
        result['pipeline'] = dict(self.diagnostics)
        if stale:
            result.update(detected=None, center_pixel=None)
        return result

    def next(self, previous):
        with self.condition:
            self.condition.wait_for(
                lambda: self.closed or (self.frame and self.frame[0] != previous), timeout=0.5)
            if self.closed or not self.frame or self.frame[0] == previous:
                return None
            # Suppress old images after input stops; status exposes the stale condition.
            if self.status()['stale']:
                return None
            return self.frame

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class VisionServer(ThreadingHTTPServer):
    daemon_threads = False
    block_on_close = True
    allow_reuse_address = True

    def __init__(self, address, store):
        self.store = store
        self.clients = set()
        self.clients_lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(8)
        super().__init__(address, VisionHandler)
        self.thread = threading.Thread(target=self.serve_forever, kwargs={'poll_interval': 0.1})
        self.thread.start()

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        with self.clients_lock:
            self.clients.add(request)
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            with self.clients_lock:
                self.clients.discard(request)
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            with self.clients_lock:
                self.clients.discard(request)
            self.slots.release()

    def close(self):
        self.store.close()
        self.shutdown()
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        self.server_close()
        self.thread.join(timeout=2)


class VisionHandler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(0.8)
        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16384)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def send_headers(self, content_type, length=None, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store, max-age=0')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Connection', 'close')
        if length is not None:
            self.send_header('Content-Length', str(length))
        self.end_headers()

    def do_GET(self):
        try:
            if self.path.split('?')[0] == '/status.json':
                payload = json.dumps(self.server.store.status(), allow_nan=False).encode()
                self.send_headers('application/json', len(payload))
                self.wfile.write(payload)
            elif self.path.split('?')[0] == '/stream.mjpg':
                self.send_headers('multipart/x-mixed-replace; boundary=frame')
                previous = 0
                while not self.server.store.closed:
                    frame = self.server.store.next(previous)
                    if frame is None:
                        readable, _, _ = select.select([self.connection], [], [], 0)
                        if readable and not self.connection.recv(1, socket.MSG_PEEK):
                            break
                        continue
                    seq, jpeg, meta, _ = frame
                    previous = seq
                    stamp = meta['stamp']['sec'] * 1000000000 + meta['stamp']['nanosec']
                    header = (f'--frame\r\nContent-Type: image/jpeg\r\n'
                              f'Content-Length: {len(jpeg)}\r\nX-Sequence: {seq}\r\n'
                              f'X-Stamp-Ns: {stamp}\r\n\r\n').encode()
                    self.wfile.write(header + jpeg + b'\r\n')
                    self.wfile.flush()
            else:
                self.send_headers('text/plain', 0, status=404)
        except (OSError, TimeoutError):
            pass


class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_http_bridge')
        bind = self.declare_parameter('bind', '0.0.0.0').value
        port = self.declare_parameter('port', 8081).value
        stale = self.declare_parameter('stale_seconds', 1.0).value
        if not 0 <= port <= 65535 or not math.isfinite(stale) or stale <= 0:
            raise ValueError('Invalid port or stale_seconds')
        self.store = LatestFrame(stale)
        self.store.diagnostics = dict(received_images=0, received_results=0,
                                      matched_frames=0, rejected_old_frames=0,
                                      last_debug_age_ms=0.0, last_encode_ms=0.0,
                                      last_detector_ms=0.0)
        self.images, self.results = collections.OrderedDict(), collections.OrderedDict()
        self.last_stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(CompressedImage, 'red_cylinder/debug_image/compressed',
                                 self.image, qos)
        self.create_subscription(String, 'red_cylinder/result', self.result,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.server = VisionServer((bind, port), self.store)
        self.get_logger().info(f'Vision HTTP listening on {bind}:{self.server.server_port}')

    def image(self, msg):
        self.store.diagnostics['received_images'] += 1
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.store.diagnostics['last_debug_age_ms'] = max(
            0.0, time.time() - (key[0] + key[1] * 1e-9)) * 1000
        self.images[key] = msg
        self.join(key)

    def result(self, msg):
        try:
            data = json.loads(msg.data)
            key = (int(data['sec']), int(data['nanosec']))
            if not isinstance(data['detected'], bool):
                return
            if not (isinstance(data['width'], int) and isinstance(data['height'], int) and
                    0 < data['width'] <= 16384 and 0 < data['height'] <= 16384):
                return
            center = data['center_pixel']
            if data['detected'] and (not isinstance(center, dict) or
                    not all(math.isfinite(float(center[k])) for k in ('u', 'v'))):
                return
            self.results[key] = data
            self.store.diagnostics['received_results'] += 1
            self.join(key)
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def join(self, key):
        if key in self.images and key in self.results:
            image = self.images.pop(key)
            data = self.results.pop(key)
            if self.last_stamp is not None and key <= self.last_stamp:
                self.store.diagnostics['rejected_old_frames'] += 1
                return
            jpeg = bytes(image.data)
            if ('jpeg' not in image.format or len(jpeg) < 4 or
                    jpeg[:2] != b'\xff\xd8' or jpeg[-2:] != b'\xff\xd9'):
                return
            if jpeg:
                self.store.diagnostics['last_encode_ms'] = data.get('jpeg_encode_ms', 0.0)
                self.store.diagnostics['last_detector_ms'] = data.get('processing_ms', 0.0)
                self.store.diagnostics['matched_frames'] += 1
                self.last_stamp = key
                age = max(0.0, time.time() - (key[0] + key[1] * 1e-9)) * 1000
                self.store.publish(jpeg, dict(
                    stamp=dict(sec=key[0], nanosec=key[1]), frame_id=image.header.frame_id,
                    width=data['width'], height=data['height'], detected=data['detected'],
                    center_pixel=data['center_pixel'] if data['detected'] else None,
                    source_age_ms=age))
        # Bound unmatched samples when one topic is absent or loses packets.
        for pending in (self.images, self.results):
            while len(pending) > 3:
                pending.popitem(last=False)

    def close(self):
        self.server.close()


def main():
    rclpy.init()
    node = None
    try:
        node = VisionBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Launch may deliver SIGINT to both the group and this process.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
