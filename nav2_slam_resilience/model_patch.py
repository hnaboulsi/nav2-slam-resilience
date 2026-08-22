"""Patch the stock TurtleBot3 LiDAR noise stddev in an unmodified vendor SDF.

The noise-severity sweep needs a way to spawn the robot with a different
LiDAR range-noise stddev per trial. `ros-jazzy-turtlebot3-gazebo` ships its
`model.sdf` with a fixed `<noise><stddev>0.01</stddev></noise>` on the
`hls_lfcd_lds` LiDAR sensor - there is no launch argument for it. Rather than
fork and hand-maintain a full copy of the vendor model, this module does a
narrow, validated text substitution on the real installed file at launch
time (see `launch/bench_headless.launch.py`): find the `hls_lfcd_lds` sensor
block specifically (the robot also has an IMU sensor with its own, unrelated
noise blocks - substituting blindly on the first `<stddev>` in the file would
silently corrupt the IMU instead), and replace its stddev.
"""

from __future__ import annotations

import re

LIDAR_SENSOR_NAME = "hls_lfcd_lds"


class ModelPatchError(ValueError):
    """Raised when the vendor SDF doesn't look like what this patch expects."""


def patch_lidar_noise_stddev(
    sdf_text: str,
    stddev: float,
    sensor_name: str = LIDAR_SENSOR_NAME,
) -> str:
    """Return `sdf_text` with the named lidar sensor's noise stddev replaced.

    Validates that exactly one matching sensor block and exactly one
    `<stddev>` inside it exist, so a future vendor SDF change that alters the
    structure fails loudly instead of silently patching the wrong element
    (or silently patching nothing).
    """
    if stddev < 0:
        raise ModelPatchError(f"stddev must be >= 0, got {stddev!r}")

    sensor_pattern = re.compile(
        rf'(<sensor\s+name="{re.escape(sensor_name)}"[^>]*>.*?</sensor>)',
        re.DOTALL,
    )
    matches = sensor_pattern.findall(sdf_text)
    if len(matches) == 0:
        raise ModelPatchError(f'no <sensor name="{sensor_name}"> block found in SDF')
    if len(matches) > 1:
        raise ModelPatchError(
            f'expected exactly one <sensor name="{sensor_name}"> block, found {len(matches)}'
        )
    sensor_block = matches[0]

    stddev_pattern = re.compile(r"<stddev>[^<]*</stddev>")
    stddev_matches = stddev_pattern.findall(sensor_block)
    if len(stddev_matches) == 0:
        raise ModelPatchError(f"no <stddev> element found inside the {sensor_name!r} sensor block")
    if len(stddev_matches) > 1:
        raise ModelPatchError(
            f"expected exactly one <stddev> element inside the {sensor_name!r} sensor block, "
            f"found {len(stddev_matches)}"
        )

    patched_block = stddev_pattern.sub(f"<stddev>{stddev}</stddev>", sensor_block, count=1)
    return sdf_text.replace(sensor_block, patched_block, 1)
