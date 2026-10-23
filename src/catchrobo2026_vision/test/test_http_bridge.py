"""Exercise actual HTTP sockets, freshness and bounded shutdown."""
import importlib.util
import collections
import json
import socket
import sys
import threading
import time
import unittest
import urllib.request
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'vision_http_bridge', Path(__file__).parents[1] / 'scripts/vision_http_bridge.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def metadata():
    now = time.time_ns()
    return dict(stamp=dict(sec=now // 1000000000, nanosec=now % 1000000000),
                frame_id='test_camera', width=32, height=24, detected=True,
                center_pixel=dict(u=16, v=12), source_age_ms=0)


class HttpTest(unittest.TestCase):
    def test_stamp_join_and_out_of_order(self):
        class Harness:
            join = bridge.VisionBridge.join
            image = bridge.VisionBridge.image
            result = bridge.VisionBridge.result
        node = Harness()
        node.images, node.results = collections.OrderedDict(), collections.OrderedDict()
        node.last_stamp = None
        node.quality = 75
        node.store = bridge.LatestFrame(1)
        node.store.diagnostics = dict(received_images=0, received_results=0, matched_frames=0,
                                      rejected_old_frames=0, last_encode_ms=0, last_debug_age_ms=0)
        def image(nanosec):
            result = bridge.CompressedImage()
            result.header.stamp.sec = int(time.time())
            result.header.stamp.nanosec = nanosec
            result.format = 'bgr8; jpeg compressed bgr8'
            result.data = b'\xff\xd8test\xff\xd9'
            return result
        def detection(msg, present):
            return bridge.String(data=json.dumps(dict(
                sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec,
                width=32, height=24, detected=present, center_pixel=dict(u=5, v=7) if present else None)))
        first, second = image(1), image(2)
        node.image(first)
        node.result(detection(second, False))
        self.assertFalse(node.store.status()['available'])
        node.result(detection(first, True))
        self.assertEqual(node.store.status()['center_pixel'], dict(u=5, v=7))
        node.image(second)
        self.assertEqual(node.store.status()['stamp']['nanosec'], 2)
        self.assertFalse(node.store.status()['detected'])
        node.image(first)
        node.result(detection(first, True))
        self.assertEqual(node.store.status()['sequence'], 2)
        for stamp in range(10, 30):
            node.image(image(stamp))
        self.assertLessEqual(len(node.images), 3)

    def test_disconnected_idle_clients_release_slots(self):
        store = bridge.LatestFrame(1)
        server = bridge.VisionServer(('127.0.0.1', 0), store)
        try:
            for _ in range(12):
                conn = socket.create_connection(server.server_address, timeout=2)
                conn.sendall(b'GET /stream.mjpg HTTP/1.0\r\n\r\n')
                self.assertIn(b'200 OK', conn.recv(1024))
                conn.close()
                deadline = time.monotonic() + 2
                while server.clients and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertFalse(server.clients)
        finally:
            server.close()

    def test_http_stale_reconnect_and_shutdown(self):
        store = bridge.LatestFrame(0.12)
        server = bridge.VisionServer(('127.0.0.1', 0), store)
        base = f'http://127.0.0.1:{server.server_port}'
        response = None
        try:
            def status():
                with urllib.request.urlopen(base + '/status.json', timeout=2) as reply:
                    self.assertEqual(reply.headers['Access-Control-Allow-Origin'], '*')
                    return json.load(reply)
            self.assertFalse(status()['available'])
            store.publish(b'jpeg', metadata())
            response = urllib.request.urlopen(base + '/stream.mjpg', timeout=2)
            self.assertEqual(response.readline().strip(), b'--frame')
            self.assertIn(b'Content-Type: image/jpeg', response.readline())
            while response.readline().strip():
                pass
            self.assertEqual(response.read(4), b'jpeg')
            response.close()
            self.assertTrue(status()['detected'])
            time.sleep(0.15)
            stale = status()
            self.assertTrue(stale['stale'])
            self.assertIsNone(stale['detected'])
            self.assertIsNone(stale['center_pixel'])
            store.publish(b'newjpeg', metadata())
            self.assertFalse(status()['stale'])
            response = urllib.request.urlopen(base + '/stream.mjpg', timeout=2)
            self.assertEqual(response.readline().strip(), b'--frame')
            # Leave a stream connected while shutdown joins all handler threads.
            started = time.monotonic()
            server.close()
            self.assertLess(time.monotonic() - started, 2)
            self.assertFalse(server.thread.is_alive())
            self.assertFalse(server.clients)
        finally:
            if response:
                response.close()
            if not store.closed:
                server.close()

    def test_latest_only(self):
        store = bridge.LatestFrame(1)
        for i in range(200):
            store.publish(bytes([i]), metadata())
        frame = store.next(1)
        self.assertEqual(frame[0], 200)
        self.assertEqual(frame[1], bytes([199]))
        self.assertIsNone(store.next(200))
        store.close()

    def test_source_age_is_stale(self):
        store = bridge.LatestFrame(1)
        meta = metadata()
        meta['source_age_ms'] = 2000
        store.publish(b'old', meta)
        self.assertTrue(store.status()['stale'])
        self.assertIsNone(store.next(0))


if __name__ == '__main__':
    unittest.main()
