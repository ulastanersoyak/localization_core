#!/usr/bin/env python3
"""Saves pole camera frames with YOLO labels computed from the true drone pose.

Run next to a simulation. Labels come from projecting the drone's bounding box
through the same pinhole model the estimator uses.
"""
import argparse
import math
import pathlib
import threading
from collections import deque

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

WIDTH, HEIGHT, FOCAL = 1920, 1080, 960.0
POLE = np.array([24.0, 0.0, 3.0])
YAW, PITCH = 3.14159265, -0.1745
# Half extents of the airframe including the rotor discs.
HALF_EXTENT = np.array([0.24, 0.33, 0.055])


def stamp(header):
    return header.stamp.sec + header.stamp.nsec * 1e-9


def world_to_camera():
    cy, sy = math.cos(YAW), math.sin(YAW)
    cp, sp = math.cos(PITCH), math.sin(PITCH)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return (rz @ ry).T


def quat_to_matrix(w, x, y, z):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def label_for(pose, r_wc):
    """YOLO label (cx, cy, w, h normalized), or None if not fully in view."""
    q = pose.orientation
    r = quat_to_matrix(q.w, q.x, q.y, q.z)
    center = np.array([pose.position.x, pose.position.y, pose.position.z])
    us, vs = [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corner = center + r @ (HALF_EXTENT * np.array([sx, sy, sz]))
                c = r_wc @ (corner - POLE)
                if c[0] <= 0.1:
                    return None
                us.append(WIDTH / 2 - FOCAL * c[1] / c[0])
                vs.append(HEIGHT / 2 - FOCAL * c[2] / c[0])
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    if u0 < 0 or v0 < 0 or u1 >= WIDTH or v1 >= HEIGHT:
        return None
    return ((u0 + u1) / 2 / WIDTH, (v0 + v1) / 2 / HEIGHT,
            (u1 - u0) / WIDTH, (v1 - v0) / HEIGHT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, type=pathlib.Path)
    ap.add_argument("--prefix", default="f")
    ap.add_argument("--every", type=int, default=3, help="keep one frame in N")
    ap.add_argument("--seconds", type=float, default=90)
    args = ap.parse_args()

    (args.out / "images").mkdir(parents=True, exist_ok=True)
    (args.out / "labels").mkdir(parents=True, exist_ok=True)

    poses = deque(maxlen=400)
    lock = threading.Lock()
    r_wc = world_to_camera()
    count = {"seen": 0, "saved": 0, "labelled": 0}
    latest = {"t": 0.0}

    def on_pose(msg):
        for p in msg.pose:
            if p.name == "drone":
                t = stamp(p.header) if p.HasField("header") else stamp(msg.header)
                with lock:
                    poses.append((t, p))
                return

    def on_image(msg):
        count["seen"] += 1
        latest["t"] = stamp(msg.header)
        if count["seen"] % args.every:
            return
        t = stamp(msg.header)
        with lock:
            if not poses:
                return
            _, pose = min(poses, key=lambda e: abs(e[0] - t))
        name = f"{args.prefix}_{count['saved']:05d}"
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        cv2.imwrite(str(args.out / "images" / f"{name}.jpg"),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        label = label_for(pose, r_wc)
        with open(args.out / "labels" / f"{name}.txt", "w") as f:
            if label:
                f.write("0 %.6f %.6f %.6f %.6f\n" % label)
                count["labelled"] += 1
        count["saved"] += 1

    node = Node()
    node.subscribe(Pose_V, "/model/drone/pose", on_pose)
    node.subscribe(Image, "/pole/camera", on_image)
    import time

    start = time.time()
    while time.time() - start < args.seconds:
        time.sleep(0.5)
    print(args.prefix, count, "last frame t=%.1f" % latest["t"])


if __name__ == "__main__":
    main()
