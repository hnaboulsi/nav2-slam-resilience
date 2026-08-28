#!/usr/bin/env python3
"""Render a recorded bag (map building + robot path) to a gif.

Sidesteps the fact that Gazebo's GUI can't render inside Docker Desktop on
Apple Silicon and GitHub Actions has no GPU either: instead of screen-
capturing a live viewer, this reconstructs the demo entirely from the bag's
own `/map` (nav_msgs/OccupancyGrid) and `/pose` (slam_toolbox's map-frame
estimate) messages, after the fact. Run inside the dev container (needs
rosbag2_py, matplotlib - both already in docker/Dockerfile).

`/map` and `/pose` don't publish anywhere near every frame we'd want to
render (slam_toolbox emits a handful of each over a short mission), so this
resamples onto a uniform real-time grid instead of one animation frame per
message: each output frame holds the most recently published map and
linearly interpolates position between the two bracketing pose samples, so
playback speed matches how long the mission actually took instead of racing
through a handful of raw messages.

Usage (inside the container):
    python3 scripts/render_bag_to_video.py \
        --bag media/bags/demo_baseline --out media/demo_baseline.gif
"""

from __future__ import annotations

import argparse
import bisect
import sys

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from nav2_slam_resilience.bagio import read_messages


def _occupancy_grid_to_rgb(msg) -> np.ndarray:
    """Render an OccupancyGrid to an (H, W, 3) uint8 image.

    Unknown cells (-1) render mid-gray; 0-100 occupancy probability renders
    white (free) to black (occupied), matching the convention every ROS
    map viewer (RViz included) uses, so the gif looks like what you'd
    actually see in RViz.
    """
    grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    rgb = np.full((*grid.shape, 3), 128, dtype=np.uint8)  # default: unknown = gray
    known = grid >= 0
    value = 255 - (grid[known].astype(np.int32) * 255 // 100)
    rgb[known] = np.stack([value, value, value], axis=-1)
    return rgb


def _load_frames(bag_path: str):
    """Return (map_frames, pose_points) sorted by recorded time.

    map_frames: list of (timestamp_ns, rgb_image, extent).
    pose_points: list of (timestamp_ns, x, y).
    """
    map_frames = []
    pose_points = []
    for bag_msg in read_messages(bag_path, topics=["/map", "/pose"]):
        if bag_msg.topic == "/map":
            info = bag_msg.msg.info
            extent = [
                info.origin.position.x,
                info.origin.position.x + info.width * info.resolution,
                info.origin.position.y,
                info.origin.position.y + info.height * info.resolution,
            ]
            map_frames.append((bag_msg.timestamp_ns, _occupancy_grid_to_rgb(bag_msg.msg), extent))
        elif bag_msg.topic == "/pose":
            p = bag_msg.msg.pose.pose.position
            pose_points.append((bag_msg.timestamp_ns, p.x, p.y))
    return map_frames, pose_points


def _interpolate_pose(pose_points, t):
    """Position at time `t`, linearly interpolated between the two nearest samples.

    Holds the first/last known pose outside the recorded range rather than
    extrapolating - a robot that hasn't started moving yet, or already
    stopped, should look stationary, not fly off in some invented direction.
    """
    if not pose_points:
        return None
    stamps = [p[0] for p in pose_points]
    i = bisect.bisect_right(stamps, t)
    if i == 0:
        return pose_points[0][1], pose_points[0][2]
    if i >= len(pose_points):
        return pose_points[-1][1], pose_points[-1][2]
    t0, x0, y0 = pose_points[i - 1]
    t1, x1, y1 = pose_points[i]
    frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)


def render(bag_path: str, out_path: str, fps: int = 8) -> None:
    map_frames, pose_points = _load_frames(bag_path)
    if not map_frames:
        raise SystemExit(f"no /map messages found in {bag_path} - was the bag actually recorded?")

    start_ts = map_frames[0][0]
    end_ts = max(map_frames[-1][0], pose_points[-1][0] if pose_points else map_frames[-1][0])
    step_ns = int(1e9 / fps)
    sample_times = list(range(start_ts, end_ts + step_ns, step_ns)) or [start_ts]
    map_stamps = [m[0] for m in map_frames]
    path_so_far: list[tuple[float, float]] = []

    fig, ax = plt.subplots(figsize=(6, 6))

    def render_frame(i: int):
        ax.clear()
        t = sample_times[i]
        map_idx = max(0, bisect.bisect_right(map_stamps, t) - 1)
        _, rgb, extent = map_frames[map_idx]
        ax.imshow(rgb, origin="lower", extent=extent)

        pos = _interpolate_pose(pose_points, t)
        if pos is not None:
            path_so_far.append(pos)
            xs, ys = zip(*path_so_far, strict=True)
            ax.plot(xs, ys, "-", color="tab:blue", linewidth=1.5)
            ax.plot(xs[-1], ys[-1], "o", color="tab:red", markersize=6)

        ax.set_title(f"t={(t - start_ts) / 1e9:.1f}s")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")

    anim = FuncAnimation(fig, render_frame, frames=len(sample_times), interval=1000 / fps)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    duration_sec = (end_ts - start_ts) / 1e9
    print(
        f"wrote {out_path}: {len(sample_times)} frames at {fps}fps "
        f"({duration_sec:.1f}s of real mission time, {len(map_frames)} /map updates, "
        f"{len(pose_points)} /pose samples)"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to the recorded bag directory")
    parser.add_argument("--out", required=True, help="Output .gif path")
    parser.add_argument("--fps", type=int, default=8)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    render(args.bag, args.out, args.fps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
