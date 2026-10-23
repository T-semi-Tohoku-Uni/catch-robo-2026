#!/usr/bin/env python3
"""Measure a generated-video -> ROS detector -> actual HTTP socket pipeline."""
import argparse
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import sys
import time
import urllib.request
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('install', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    install = args.install.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parents[1]
    video = output / 'moving-cylinder.avi'
    subprocess.run([sys.executable, str(source / 'scripts/make_test_video.py'), str(video)], check=True)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    namespace = '/vision_probe_' + uuid.uuid4().hex[:8]
    env = dict(os.environ, ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1')
    processes, logs = [], []
    base = f'http://127.0.0.1:{port}'

    def start(package, executable, params=()):
        binary = install / package / 'lib' / package / executable
        log = open(output / (executable + f'-{len(logs)}.log'), 'w')
        logs.append(log)
        proc = subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:=' + namespace,
                                 *params], env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append(proc)
        return proc

    def stop(proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)

    def camera():
        return start('camera_publisher', 'camera_publisher_node',
                     ['-p', 'backend:=video', '-p', f'video_path:={video}', '-p', 'fps:=20.0'])

    def status():
        with urllib.request.urlopen(base + '/status.json', timeout=2) as response:
            return json.load(response)

    def wait_fresh():
        until = time.monotonic() + 12
        while time.monotonic() < until:
            try:
                result = status()
                if result['available'] and not result['stale']:
                    return result
            except OSError:
                pass
            time.sleep(0.05)
        raise AssertionError('No fresh HTTP recognition result')

    def connection(small_window=False):
        client = http.client.HTTPConnection('127.0.0.1', port, timeout=3)
        client.connect()
        if small_window:
            client.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        client.request('GET', '/stream.mjpg')
        response = client.getresponse()
        assert response.status == 200
        return client, response

    def frame(response):
        line = response.readline()
        while line in (b'\r\n', b'\n'):
            line = response.readline()
        if not line:
            raise EOFError('MJPEG client disconnected')
        assert line.strip() == b'--frame', line
        headers = {}
        while True:
            line = response.readline()
            if not line.strip():
                break
            name, value = line.decode().split(':', 1)
            headers[name.lower()] = value.strip()
        length = int(headers['content-length'])
        jpeg = response.read(length)
        assert len(jpeg) == length and jpeg[:2] == b'\xff\xd8' and jpeg[-2:] == b'\xff\xd9'
        return dict(sequence=int(headers['x-sequence']), bytes=length,
                    latency_ms=(time.time_ns() - int(headers['x-stamp-ns'])) / 1e6,
                    received=time.monotonic())

    try:
        bridge = start('catchrobo2026_vision', 'vision_http_bridge.py',
                       ['-p', f'port:={port}', '-p', 'bind:=127.0.0.1'])
        detector = start('red_cylinder_detector', 'red_cylinder_detector_node')
        cam = camera()
        first = wait_fresh()
        client, response = connection()
        frames = []
        begin = time.monotonic()
        while time.monotonic() - begin < 3:
            frames.append(frame(response))
        response.close()
        client.close()
        last = status()
        elapsed = frames[-1]['received'] - frames[0]['received']
        delays = sorted(f['latency_ms'] for f in frames)
        summary = dict(
            transport='actual HTTP over loopback; not physical LAN/Jetson measurement',
            frames=len(frames), fps=(len(frames) - 1) / elapsed,
            jpeg_mbit_s=sum(f['bytes'] for f in frames[1:]) * 8 / elapsed / 1e6,
            latency_ms=dict(median=statistics.median(delays), p95=delays[int(len(delays) * 0.95)],
                            maximum=max(delays)), first_status=first)
        (output / 'throughput.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2), flush=True)
        summary['pipeline_at_end'] = last.get('pipeline')
        # A constrained receive window must not block the normal status endpoint.
        client, response = connection(small_window=True)
        slow = [frame(response)]
        until = time.monotonic() + 4
        disconnected = False
        while time.monotonic() < until:
            time.sleep(0.22)
            try:
                slow.append(frame(response))
            except (EOFError, OSError, http.client.HTTPException):
                disconnected = True
                break
        response.close()
        client.close()
        summary['slow_client'] = dict(sequences=[f['sequence'] for f in slow],
                                      disconnected=disconnected,
                                      maximum_gap=max([b['sequence'] - a['sequence']
                                                       for a, b in zip(slow, slow[1:])] or [0]))
        fresh = status()
        client, response = connection()
        reconnected = frame(response)
        response.close()
        client.close()
        assert reconnected['sequence'] >= fresh['sequence']
        summary['reconnected_sequence'] = reconnected['sequence']
        # Stop only this probe's video producer, retaining detector and HTTP server.
        stop(cam)
        time.sleep(1.25)
        stale = status()
        assert stale['stale'] and stale['detected'] is None and stale['center_pixel'] is None
        summary['stale_status'] = stale
        cam = camera()
        resumed = wait_fresh()
        assert resumed['sequence'] > stale['sequence']
        summary['resumed_status'] = resumed
        assert bridge.poll() is None and detector.poll() is None
        assert summary['fps'] > 10 and summary['latency_ms']['p95'] < 500
        # A synthetic video's no-object interval is a fresh negative, not stale data.
        until = time.monotonic() + 6
        negative = None
        while time.monotonic() < until:
            sample = status()
            if sample['detected'] is False and not sample['stale']:
                negative = sample
                break
            time.sleep(0.1)
        assert negative is not None, 'No fresh negative detection during video absence interval'
        summary['negative_status'] = negative
        (output / 'measurement.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    finally:
        for proc in reversed(processes):
            stop(proc)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
