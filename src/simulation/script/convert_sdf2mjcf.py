#!/usr/bin/env python3
"""Convert the robot arm SDF model to MJCF.

MuJoCo requires bodies to form a tree.  SDF models can express closed
kinematic loops directly by giving a link multiple joint parents.  This
converter keeps the first parent joint in the MJCF body tree and converts
additional revolute joints into equality/connect constraints at their pivots.
That gives MuJoCo and Genesis a usable closed-loop approximation while keeping
the original visual, collision, mass, and inertia data.
"""

from __future__ import annotations

import argparse
import math
import posixpath
import re
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "robot_arm" / "model.sdf"
DEFAULT_OUTPUT = ROOT / "robot_arm" / "model.xml"
MAX_STL_FACES = 200_000
DEFAULT_MESH_SCALE = 0.001


def parse_float_list(text: str | None, expected: int | None = None) -> list[float]:
    values = [float(x) for x in (text or "").split()]
    if expected is not None and len(values) != expected:
        raise ValueError(f"Expected {expected} floats, got {len(values)}: {text!r}")
    return values


def fmt(values: Iterable[float]) -> str:
    return " ".join(f"{v:.12g}" for v in values)


def safe_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def rotmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def transpose3(r: list[list[float]]) -> list[list[float]]:
    return [[r[j][i] for j in range(3)] for i in range(3)]


def transform_vec(r: list[list[float]], v: list[float]) -> list[float]:
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def pose_to_matrix(pose: list[float]) -> list[list[float]]:
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    r = rotmul(rotmul(rz, ry), rx)
    return [
        [r[0][0], r[0][1], r[0][2], x],
        [r[1][0], r[1][1], r[1][2], y],
        [r[2][0], r[2][1], r[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def invert_transform(t: list[list[float]]) -> list[list[float]]:
    r = [row[:3] for row in t[:3]]
    rt = transpose3(r)
    p = [t[0][3], t[1][3], t[2][3]]
    tinv = [[0.0] * 4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            tinv[i][j] = rt[i][j]
        tinv[i][3] = -sum(rt[i][j] * p[j] for j in range(3))
    tinv[3][3] = 1.0
    return tinv


def matrix_to_pos_quat(t: list[list[float]]) -> tuple[list[float], list[float]]:
    r = [row[:3] for row in t[:3]]
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    quat = [qw / norm, qx / norm, qy / norm, qz / norm]
    pos = [t[0][3], t[1][3], t[2][3]]
    return pos, quat


def rotate_inertia_to_body(link: Link) -> list[float]:
    inertia = [
        [
            link.inertia.get("ixx", 0.0),
            link.inertia.get("ixy", 0.0),
            link.inertia.get("ixz", 0.0),
        ],
        [
            link.inertia.get("ixy", 0.0),
            link.inertia.get("iyy", 0.0),
            link.inertia.get("iyz", 0.0),
        ],
        [
            link.inertia.get("ixz", 0.0),
            link.inertia.get("iyz", 0.0),
            link.inertia.get("izz", 0.0),
        ],
    ]
    inertial_frame = pose_to_matrix(link.inertial_pose or [0.0] * 6)
    r = [row[:3] for row in inertial_frame[:3]]
    rotated = rotmul(rotmul(r, inertia), transpose3(r))
    return [
        rotated[0][0],
        rotated[1][1],
        rotated[2][2],
        rotated[0][1],
        rotated[0][2],
        rotated[1][2],
    ]


def child_frame_axis(joint_pose: list[float], axis: list[float]) -> list[float]:
    r = [row[:3] for row in pose_to_matrix(joint_pose)[:3]]
    axis_child = transform_vec(r, axis)
    norm = math.sqrt(sum(v * v for v in axis_child))
    return [v / norm for v in axis_child]


def world_point(body_world: list[list[float]], local_point: list[float]) -> list[float]:
    return [
        sum(body_world[i][j] * local_point[j] for j in range(3)) + body_world[i][3]
        for i in range(3)
    ]


@dataclass
class Link:
    name: str
    pose: list[float]
    meshes: list[str]
    mass: float | None
    inertial_pose: list[float] | None
    inertia: dict[str, float] = field(default_factory=dict)


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    pose: list[float]
    axis: list[float]


def text_of(parent: ET.Element, path: str, default: str | None = None) -> str | None:
    found = parent.find(path)
    if found is None or found.text is None:
        return default
    return found.text.strip()


def source_mesh_path(uri: str | None) -> Path | None:
    if not uri:
        return None
    if uri.startswith("model://robot_arm/"):
        return ROOT / "robot_arm" / uri.removeprefix("model://robot_arm/")
    elif uri.startswith("model://"):
        return ROOT / uri.split("/", 3)[-1]
    return Path(uri)


def rel_asset_path(source: Path, output_dir: Path) -> str:
    return posixpath.relpath(source, output_dir).replace("\\", "/")


def stl_face_count(source: Path) -> int | None:
    data = source.read_bytes()
    if len(data) < 84:
        return None
    face_count = struct.unpack("<I", data[80:84])[0]
    if len(data) == 84 + face_count * 50:
        return face_count
    return None


def split_binary_stl(source: Path, output_dir: Path, mesh_name: str) -> list[Path]:
    data = source.read_bytes()
    header = data[:80]
    face_count = struct.unpack("<I", data[80:84])[0]
    triangles = data[84:]
    mesh_dir = output_dir / "mjcf_meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    parts: list[Path] = []
    for index, start in enumerate(range(0, face_count, MAX_STL_FACES)):
        count = min(MAX_STL_FACES, face_count - start)
        part = mesh_dir / f"{mesh_name}_part{index}.stl"
        payload = triangles[start * 50 : (start + count) * 50]
        part_header = header[:60] + f" split {index}".encode("ascii")
        part_header = part_header[:80].ljust(80, b" ")
        part.write_bytes(part_header + struct.pack("<I", count) + payload)
        parts.append(part)
    return parts


def prepare_mesh_assets(source: Path | None, output_dir: Path, mesh_name: str) -> list[str]:
    if source is None:
        return []
    face_count = stl_face_count(source)
    if face_count is None or face_count <= MAX_STL_FACES:
        return [rel_asset_path(source, output_dir)]

    parts = split_binary_stl(source, output_dir, mesh_name)
    return [rel_asset_path(part, output_dir) for part in parts]


def parse_sdf(path: Path, output_dir: Path) -> tuple[str, dict[str, Link], list[Joint]]:
    tree = ET.parse(path)
    model = tree.getroot().find("model")
    if model is None:
        raise ValueError("SDF does not contain a <model> element")

    links: dict[str, Link] = {}
    for element in model.findall("link"):
        name = element.attrib["name"]
        inertial = element.find("inertial")
        inertia_element = element.find("inertial/inertia")
        inertia = {}
        if inertia_element is not None:
            for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                inertia[key] = float(text_of(inertia_element, key, "0") or "0")
        uri = text_of(element, "visual/geometry/mesh/uri") or text_of(
            element, "collision/geometry/mesh/uri"
        )
        link_mesh_name = safe_name(name)
        links[name] = Link(
            name=name,
            pose=parse_float_list(text_of(element, "pose", "0 0 0 0 0 0"), 6),
            meshes=prepare_mesh_assets(source_mesh_path(uri), output_dir, link_mesh_name),
            mass=float(text_of(inertial, "mass", "0") or "0") if inertial is not None else None,
            inertial_pose=parse_float_list(text_of(inertial, "pose", "0 0 0 0 0 0"), 6)
            if inertial is not None
            else None,
            inertia=inertia,
        )

    joints: list[Joint] = []
    for element in model.findall("joint"):
        kind = element.attrib.get("type", "revolute")
        if kind not in {"revolute", "continuous", "fixed"}:
            raise ValueError(f"Unsupported joint type {kind!r} in {element.attrib.get('name')}")
        joints.append(
            Joint(
                name=element.attrib["name"],
                kind=kind,
                parent=text_of(element, "parent") or "",
                child=text_of(element, "child") or "",
                pose=parse_float_list(text_of(element, "pose", "0 0 0 0 0 0"), 6),
                axis=parse_float_list(text_of(element, "axis/xyz", "0 0 1"), 3),
            )
        )

    return model.attrib.get("name", path.stem), links, joints


def add_mesh_assets(
    root: ET.Element, links: dict[str, Link], mesh_scale: float
) -> dict[str, list[str]]:
    asset = ET.SubElement(root, "asset")
    mesh_names: dict[str, list[str]] = {}
    for link in links.values():
        if not link.meshes:
            continue
        mesh_names[link.name] = []
        for index, mesh_file in enumerate(link.meshes):
            mesh_name = safe_name(link.name)
            if len(link.meshes) > 1:
                mesh_name = f"{mesh_name}_part{index}"
            mesh_names[link.name].append(mesh_name)
            attributes = {"name": mesh_name, "file": mesh_file}
            if mesh_scale != 1.0:
                attributes["scale"] = fmt([mesh_scale, mesh_scale, mesh_scale])
            ET.SubElement(asset, "mesh", **attributes)
    return mesh_names


def add_inertial(body: ET.Element, link: Link) -> None:
    if link.mass is None or link.inertial_pose is None:
        return
    pos, _quat = matrix_to_pos_quat(pose_to_matrix(link.inertial_pose))
    ET.SubElement(
        body,
        "inertial",
        pos=fmt(pos),
        mass=f"{link.mass:.12g}",
        fullinertia=fmt(rotate_inertia_to_body(link)),
    )


def add_link_geoms(body: ET.Element, link: Link, mesh_names: dict[str, list[str]]) -> None:
    names = mesh_names.get(link.name, [])
    if not names:
        return
    for index, mesh_name in enumerate(names):
        suffix = f"_part{index}" if len(names) > 1 else ""
        ET.SubElement(
            body,
            "geom",
            name=f"{safe_name(link.name)}_visual{suffix}",
            type="mesh",
            mesh=mesh_name,
            contype="0",
            conaffinity="0",
            group="1",
        )
        ET.SubElement(
            body,
            "geom",
            name=f"{safe_name(link.name)}_collision{suffix}",
            type="mesh",
            mesh=mesh_name,
            group="3",
        )


def select_tree(
    links: dict[str, Link], joints: list[Joint]
) -> tuple[list[str], dict[str, list[Joint]], list[Joint]]:
    tree_children: dict[str, list[Joint]] = {name: [] for name in links}
    closure_joints: list[Joint] = []
    tree_parent: dict[str, str] = {}

    def would_create_cycle(parent: str, child: str) -> bool:
        cursor = parent
        while cursor in tree_parent:
            if cursor == child:
                return True
            cursor = tree_parent[cursor]
        return cursor == child

    for joint in joints:
        if joint.parent not in links or joint.child not in links:
            continue
        if joint.child in tree_parent or would_create_cycle(joint.parent, joint.child):
            closure_joints.append(joint)
            continue
        tree_parent[joint.child] = joint.parent
        tree_children[joint.parent].append(joint)

    roots = [name for name in links if name not in tree_parent]
    if not roots and links:
        roots = [next(iter(links))]

    return roots, tree_children, closure_joints


def convert(
    input_path: Path,
    output_path: Path,
    floating_root: bool = False,
    mesh_scale: float = DEFAULT_MESH_SCALE,
) -> list[Joint]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    model_name, links, joints = parse_sdf(input_path, output_path.parent)
    world_poses = {name: pose_to_matrix(link.pose) for name, link in links.items()}
    roots, tree_children, closure_joints = select_tree(links, joints)

    mjcf = ET.Element("mujoco", model=model_name)
    ET.SubElement(
        mjcf,
        "compiler",
        angle="radian",
        meshdir=".",
        autolimits="true",
        balanceinertia="true",
    )
    ET.SubElement(mjcf, "option", timestep="0.001", integrator="implicitfast")
    default = ET.SubElement(mjcf, "default")
    ET.SubElement(default, "joint", damping="0.05", armature="0.001")
    ET.SubElement(default, "geom", density="0", friction="0.8 0.1 0.1")

    mesh_names = add_mesh_assets(mjcf, links, mesh_scale)
    worldbody = ET.SubElement(mjcf, "worldbody")

    def add_body(parent_xml: ET.Element, link_name: str, parent_link: str | None) -> None:
        link = links[link_name]
        local = (
            world_poses[link_name]
            if parent_link is None
            else matmul(invert_transform(world_poses[parent_link]), world_poses[link_name])
        )
        pos, quat = matrix_to_pos_quat(local)
        body = ET.SubElement(parent_xml, "body", name=link.name, pos=fmt(pos), quat=fmt(quat))

        add_inertial(body, link)
        if parent_link is None and floating_root:
            ET.SubElement(body, "freejoint", name=f"{safe_name(link.name)}_free")
        add_link_geoms(body, link, mesh_names)

        for joint in tree_children.get(link_name, []):
            child = links[joint.child]
            child_local = matmul(invert_transform(world_poses[link_name]), world_poses[child.name])
            child_pos, child_quat = matrix_to_pos_quat(child_local)
            child_body = ET.SubElement(
                body, "body", name=child.name, pos=fmt(child_pos), quat=fmt(child_quat)
            )
            if joint.kind != "fixed":
                ET.SubElement(
                    child_body,
                    "joint",
                    name=joint.name,
                    type="hinge",
                    pos=fmt(joint.pose[:3]),
                    axis=fmt(child_frame_axis(joint.pose, joint.axis)),
                )
            add_inertial(child_body, child)
            add_link_geoms(child_body, child, mesh_names)

            def add_descendants(current_body: ET.Element, current_link: str) -> None:
                for child_joint in tree_children.get(current_link, []):
                    next_link = links[child_joint.child]
                    next_local = matmul(
                        invert_transform(world_poses[current_link]), world_poses[next_link.name]
                    )
                    next_pos, next_quat = matrix_to_pos_quat(next_local)
                    next_body = ET.SubElement(
                        current_body,
                        "body",
                        name=next_link.name,
                        pos=fmt(next_pos),
                        quat=fmt(next_quat),
                    )
                    if child_joint.kind != "fixed":
                        ET.SubElement(
                            next_body,
                            "joint",
                            name=child_joint.name,
                            type="hinge",
                            pos=fmt(child_joint.pose[:3]),
                            axis=fmt(child_frame_axis(child_joint.pose, child_joint.axis)),
                        )
                    add_inertial(next_body, next_link)
                    add_link_geoms(next_body, next_link, mesh_names)
                    add_descendants(next_body, next_link.name)

            add_descendants(child_body, child.name)

    for root in roots:
        add_body(worldbody, root, None)

    if closure_joints:
        equality = ET.SubElement(mjcf, "equality")
        for joint in closure_joints:
            anchor = world_point(world_poses[joint.child], joint.pose[:3])
            ET.SubElement(
                equality,
                "connect",
                name=f"{joint.name}_closed_loop",
                body1=joint.parent,
                body2=joint.child,
                anchor=fmt(anchor),
                solref="0.02 1",
                solimp="0.9 0.95 0.001",
            )

    ET.indent(mjcf, space="  ")
    ET.ElementTree(mjcf).write(output_path, encoding="utf-8", xml_declaration=True)
    return closure_joints


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--floating-root",
        action="store_true",
        help="add a freejoint to root bodies instead of fixing them to the world",
    )
    parser.add_argument(
        "--mesh-scale",
        type=float,
        default=DEFAULT_MESH_SCALE,
        help="uniform scale applied to mesh assets; CAD-exported STL files are often in mm",
    )
    args = parser.parse_args()

    closures = convert(args.input, args.output, args.floating_root, args.mesh_scale)
    print(f"Wrote {args.output}")
    if closures:
        names = ", ".join(joint.name for joint in closures)
        print(f"Converted closed-loop joints to equality/connect constraints: {names}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
