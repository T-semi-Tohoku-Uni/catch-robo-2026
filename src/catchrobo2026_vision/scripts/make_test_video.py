#!/usr/bin/env python3
"""Generate a deterministic moving cylinder clip with absent/wrong-color intervals."""
import argparse
import math
from pathlib import Path
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--fps', type=float, default=20.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not math.isfinite(args.fps) or args.seconds <= 0 or args.fps <= 0:
        parser.error('seconds and fps must be positive finite values')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*'MJPG'), args.fps, (640, 480))
    if not writer.isOpened():
        raise RuntimeError('MJPG VideoWriter could not open output')
    for index in range(round(args.seconds * args.fps)):
        frame = np.full((480, 640, 3), 35, dtype=np.uint8)
        phase = (index / args.fps) % 6
        x = round(275 + 170 * math.sin(index / args.fps))
        if phase < 4 or phase >= 5:
            color = (0, 0, 230) if phase < 4 else (0, 220, 0)
            cv2.rectangle(frame, (x, 150), (x + 65, 320), color, -1)
            cv2.ellipse(frame, (x + 32, 150), (32, 9), 0, 0, 360, color, -1)
        cv2.putText(frame, f'SIMULATED frame {index:05d}', (16, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
        writer.write(frame)
    writer.release()
    print(f'{args.output}: {round(args.seconds * args.fps)} frames at {args.fps} FPS')


if __name__ == '__main__':
    main()
