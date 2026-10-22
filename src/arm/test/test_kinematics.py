"""Evaluate expanded URDF transforms against the IK's field-frame geometry."""
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'arm_joints', ROOT / 'scripts' / 'arm_joint_state_publisher.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] = (np.eye(3) * math.cos(angle)
                       + (1 - math.cos(angle)) * np.outer(axis, axis)
                       + math.sin(angle) * skew)
    return result


def urdf_frames(angles):
    root = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/robot_arm.xacro')).toxml())
    values = dict(zip(bridge.JOINT_NAMES, bridge.urdf_positions(angles)))
    frames = {'map': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
                roll, pitch, yaw = map(float, origin.get('rpy', '0 0 0').split())
                transform = (transform @ rotation([0, 0, 1], yaw)
                             @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll))
            if joint.get('type') != 'fixed':
                axis = list(map(float, joint.find('axis').get('xyz').split()))
                transform = transform @ rotation(axis, values[joint.get('name')])
            frames[joint.find('child').get('link')] = frames[parent] @ transform
            pending.remove(joint)
            progress = True
        assert progress, 'URDF contains a disconnected or cyclic joint tree'
    return frames


def test_field_geometry():
    rng = np.random.default_rng(2026)
    for angles in [[0, 0, 0, 0], *rng.uniform(-math.pi, math.pi, (100, 4))]:
        t1, t2, t3, t4 = angles
        frames = urdf_frames(angles)
        radial = np.array([math.sin(t1), math.cos(t1), 0])
        vertical = np.array([0, 0, 1])
        base = np.array([0.675, -0.190, 0])
        shoulder = base - 0.040 * radial + 0.090 * vertical
        elbow = shoulder + 0.480 * (math.sin(t2) * radial + math.cos(t2) * vertical)
        wrist = elbow + 0.480 * (math.sin(t3) * radial + math.cos(t3) * vertical)
        for name, position in zip(
                ['base_link', 'base_yaw_link', 'upper_arm_link', 'forearm_link',
                 'wrist_link', 'flange_link'],
                [base, base, shoulder, elbow, wrist, wrist + 0.040 * radial]):
            np.testing.assert_allclose(frames[name][:3, 3], position, atol=1e-12)
        np.testing.assert_allclose(frames['flange_link'][:3, :3],
                                   rotation([0, 0, 1], t1 + t4)[:3, :3], atol=1e-12)
        np.testing.assert_allclose(frames['wrist_link'][:3, 2], vertical, atol=1e-12)


def test_collision_geometry():
    root = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/robot_arm.xacro')).toxml())
    bodies = ['base_yaw_link', 'upper_arm_link', 'forearm_link', 'wrist_link', 'flange_link']
    for name in bodies:
        collision = root.find(f"link[@name='{name}']/collision/geometry")
        assert collision is not None
        assert collision.find('mesh') is None
        for geometry in collision:
            for value in geometry.attrib.values():
                assert all(float(dimension) > 0 for dimension in value.split())


@pytest.mark.parametrize('angles', [[0, 0, 0], [0, 0, 0, float('nan')],
                                    [0, 0, float('inf'), 0]])
def test_invalid_angles(angles):
    with pytest.raises(ValueError):
        bridge.urdf_positions(angles)
