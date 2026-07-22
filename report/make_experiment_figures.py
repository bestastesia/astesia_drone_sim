#!/usr/bin/env python3
"""Extract Jazzy rosbag2 CDR messages and make report figures.

Usage: python3 make_experiment_figures.py BAG_DIR OUTPUT_DIR
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
                ConverterOptions("", "cdr"))
    types = {t.name: get_message(t.type)
             for t in reader.get_all_topics_and_types()}
    rows, goals, obstacles = [], [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(raw, types[topic])
        t = stamp * 1e-9
        if topic == "/drone/odom":
            p = msg.pose.pose.position
            rows.append((t, p.x, p.y, p.z))
        elif topic == "/drone/goal":
            p = msg.pose.position
            goals.append((t, p.x, p.y, p.z))
        elif topic == "/map/obstacles":
            obstacles = [(m.pose.position.x, m.pose.position.y,
                          max(m.scale.x, m.scale.y) / 2.0)
                         for m in msg.markers if m.action != 2]
    if not rows:
        raise RuntimeError("bag contains no /drone/odom messages")
    t0 = rows[0][0]
    return [(t - t0, x, y, z) for t, x, y, z in rows], goals, obstacles


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: make_experiment_figures.py BAG_DIR OUTPUT_DIR")
    data, goals, obstacles = read_bag(sys.argv[1])
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    t = [r[0] for r in data]
    x = [r[1] for r in data]
    y = [r[2] for r in data]
    z = [r[3] for r in data]
    gx, gy, gz = (goals[-1][1:] if goals else (0.0, 0.0, 1.5))
    ex = [xx - gx for xx in x]
    ey = [yy - gy for yy in y]
    ez = [zz - gz for zz in z]
    dmin = []
    for xx, yy in zip(x, y):
        if obstacles:
            dmin.append(min(math.hypot(xx - ox, yy - oy) - r - 0.2
                             for ox, oy, r in obstacles))
        else:
            dmin.append(float("nan"))
    with open(os.path.join(out, "square_metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "ex", "ey", "ez", "min_obstacle_distance"])
        w.writerows(zip(t, x, y, z, ex, ey, ez, dmin))

    plt.figure(figsize=(6.4, 4.0))
    plt.plot(x, y, label="measured trajectory", lw=1.5)
    if goals:
        plt.scatter([g[1] for g in goals], [g[2] for g in goals],
                    s=16, label="goal samples")
    for ox, oy, r in obstacles:
        plt.scatter([ox], [oy], c="tab:red", s=30)
        plt.annotate("obstacle", (ox, oy), fontsize=7)
    plt.xlabel("x [m]"); plt.ylabel("y [m]"); plt.axis("equal")
    plt.grid(alpha=.25); plt.legend(fontsize=8); plt.tight_layout()
    plt.savefig(os.path.join(out, "trajectory_xy.png"), dpi=180)
    plt.close()

    plt.figure(figsize=(6.4, 4.0))
    plt.plot(t, ex, label="$e_x$"); plt.plot(t, ey, label="$e_y$")
    plt.plot(t, ez, label="$e_z$")
    plt.axhline(0, color="k", lw=.5); plt.xlabel("time [s]")
    plt.ylabel("position error [m]"); plt.grid(alpha=.25)
    plt.legend(); plt.tight_layout()
    plt.savefig(os.path.join(out, "position_error.png"), dpi=180)
    plt.close()

    plt.figure(figsize=(6.4, 4.0))
    plt.plot(t, dmin, color="tab:orange", label="clearance")
    plt.axhline(.4, color="tab:red", ls="--", label="safety threshold 0.4 m")
    plt.xlabel("time [s]"); plt.ylabel("clearance [m]")
    plt.grid(alpha=.25); plt.legend(); plt.tight_layout()
    plt.savefig(os.path.join(out, "obstacle_clearance.png"), dpi=180)
    plt.close()
    print(f"samples={len(data)} goals={len(goals)} obstacles={len(obstacles)}")
    print(f"final=({x[-1]:.3f},{y[-1]:.3f},{z[-1]:.3f}) min_clearance={min(dmin):.3f}m")


if __name__ == "__main__":
    main()
