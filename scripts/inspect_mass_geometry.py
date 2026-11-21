#!/usr/bin/env python3
"""Read-only ERP42 URDF mass distribution and COLLADA collision-mesh bounds.

Run with:
  python3 scripts/inspect_mass_geometry.py

Requires ROS 2 Humble's xacro executable, numpy, and lxml (as installed in
the Ubuntu-22.04 environment). No simulator or source files are modified.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from lxml import etree


ROOT = Path(__file__).resolve().parents[1]
URDF_XACRO = ROOT / "erp42/urdf/erp42.urdf.xacro"
MESH = ROOT / "erp42/meshes/erp42_structure.dae"
if not URDF_XACRO.is_file():
    share = Path("/root/erp42_ws/install/erp42/share/erp42")
    URDF_XACRO = share / "urdf/erp42.urdf.xacro"
    MESH = share / "meshes/erp42_structure.dae"
NS = "{http://www.collada.org/2005/11/COLLADASchema}"


def xyz(text: str | None):
    return np.array([float(s) for s in (text or "0 0 0").split()], dtype=float)


def transform(origin=None):
    xyz_value = xyz(origin.get("xyz")) if origin is not None else np.zeros(3)
    r, p, y = xyz(origin.get("rpy")) if origin is not None else np.zeros(3)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = xyz_value
    return matrix


def expand_urdf():
    # Run with the normal ROS setup because xacro lives on ROS's PYTHONPATH.
    run = subprocess.run(
        ["bash", "-c", 'source /opt/ros/humble/setup.bash; source /root/erp42_ws/install/setup.bash; exec xacro "$1"', "bash", str(URDF_XACRO)],
        capture_output=True, text=True,
    )
    if run.returncode:
        raise RuntimeError(f"xacro failed (code {run.returncode}):\n{run.stderr[-4000:]}")
    return ET.fromstring(run.stdout)


def inspect_mass(robot):
    links = {}
    parents = {}
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            mass, offset = 0.0, np.zeros(3)
        else:
            mass = float(inertial.find("mass").get("value"))
            offset = transform(inertial.find("origin"))[:3, 3]
        links[link.get("name")] = (mass, offset)
    for joint in robot.findall("joint"):
        child = joint.find("child").get("link")
        parent = joint.find("parent").get("link")
        parents[child] = (parent, transform(joint.find("origin")))

    poses = {"structure": np.eye(4)}

    def world_pose(name):
        if name not in poses:
            parent, relative = parents[name]
            poses[name] = world_pose(parent) @ relative
        return poses[name]

    total = 0.0
    mass_point = np.zeros(3)
    rows = []
    for name, (mass, offset) in links.items():
        if mass <= 0:
            continue
        pos = (world_pose(name) @ np.append(offset, 1.0))[:3]
        rows.append((name, mass, pos))
        total += mass
        mass_point += mass * pos
    com = mass_point / total
    front, rear = 0.53, -0.49
    rear_ratio = (front - com[0]) / (front - rear)
    print(f"TOTAL_MASS_KG={total:.6f}")
    print("COM_IN_STRUCTURE_M:", " ".join(f"{v:+.6f}" for v in com))
    print(f"FRONT_LOAD_FRACTION={1-rear_ratio:.6f}; REAR_LOAD_FRACTION={rear_ratio:.6f}")
    for name, mass, pos in sorted(rows):
        print(f"  {name}: mass={mass:.6g} kg, center={pos.round(5).tolist()}")
    return robot


def inspect_mesh(robot):
    # Extract just the positions instead of parsing the huge triangle list.
    with open(MESH, "rb") as file:
        for _, elem in etree.iterparse(
            file, events=("end",), tag=f"{NS}float_array",
        ):
            if "positions-array" in elem.get("id", ""):
                vertices = np.fromstring(elem.text, sep=" ", dtype=np.float64).reshape(-1, 3)
                elem.clear()
                break
            elem.clear()
        else:
            raise RuntimeError("Positions source not found")

    # Blender exported the geometry with a node transform, not unit-scale vertices.
    # Avoid parsing the 60 MB COLLADA a second time: its node matrix is near EOF.
    import re
    with open(MESH, "rb") as file:
        file.seek(-8192, os.SEEK_END)
        tail = file.read().decode("utf-8", errors="replace")
    match = re.search(r'<matrix sid="transform">([^<]+)</matrix>', tail)
    if match is None:
        raise RuntimeError("COLLADA scene node matrix not found near EOF")
    collada = np.array([float(x) for x in match.group(1).split()]).reshape(4, 4).T
    urdf_collision = transform(robot.find("./link[@name='structure']/collision/origin"))
    pts = (urdf_collision @ collada @ np.column_stack((vertices, np.ones(len(vertices)))).T).T[:, :3]
    low = pts.min(axis=0)
    high = pts.max(axis=0)
    print("COLLADA_NODE_TRANSFORM:", collada.tolist())
    print("COLLISION_MESH_BOUNDS_STRUCTURE_M:", "min", low.round(4), "max", high.round(4))
    print(f"SPAWN_Z=0.32; WHEEL_RADIUS=0.28; nominal wheel-bottom clearance=0.04 m")
    print(f"SPAWN_MESH_MIN_WORLD_Z={0.32 + low[2]:.4f} m")
    print("COLLISION_MESH_LOWEST_POINTS:")
    order = np.argsort(pts[:, 2])[:15]
    for i in order[::3]:
        print(f"  x/y/z={pts[i].round(4).tolist()}")
    print("MIN_HEIGHT_BY_X_BIN (relative to structure origin):")
    bins = np.linspace(low[0], high[0], 11)
    for a, b in zip(bins[:-1], bins[1:]):
        subset = pts[(pts[:, 0] >= a) & (pts[:, 0] < b)]
        if len(subset):
            print(f"  x={a:+.3f} .. {b:+.3f}: z_min={subset[:,2].min():+.4f}; count={len(subset)}")


if __name__ == "__main__":
    print("URDF_XACRO=", URDF_XACRO)
    print("COLLADA_MESH=", MESH)
    robot = expand_urdf()
    inspect_mass(robot)
    inspect_mesh(robot)
