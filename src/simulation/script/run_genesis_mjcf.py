#!/usr/bin/env python3
"""Run the converted MJCF robot arm in Genesis."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import genesis as gs
import torch


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "robot_arm" / "model.xml"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--backend", choices=("cpu", "gpu"), default="gpu")
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--collision", action="store_true")
    parser.add_argument("--visualization", action="store_true")
    parser.add_argument(
        "--hold",
        action="store_true",
        help="keep the process alive after build/steps; press Ctrl-C to quit",
    )
    args = parser.parse_args()

    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

    print(f"torch: {torch.__version__}")
    print(f"torch cuda available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"cuda device: {torch.cuda.get_device_name(0)}")

    backend = gs.gpu if args.backend == "gpu" else gs.cpu
    gs.init(backend=backend, logging_level="warning")

    scene = gs.Scene(show_viewer=args.viewer)
    robot = scene.add_entity(
        gs.morphs.MJCF(
            file=str(args.model),
            collision=args.collision,
            visualization=args.visualization,
        )
    )
    scene.build()
    for _ in range(args.steps):
        scene.step()

    print(f"genesis backend requested: {args.backend}")
    print(f"entity: {type(robot).__name__}")
    print(f"steps: {args.steps}")
    if args.hold:
        print("holding viewer; press Ctrl-C to quit")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
