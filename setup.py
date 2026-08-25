import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nav2_slam_resilience"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "robot"), glob("robot/*.xacro")),
        (os.path.join("share", package_name, "scenarios"), glob("scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hassan Naboulsi",
    maintainer_email="hnaboulsi31@gmail.com",
    description=(
        "A quantitative robustness benchmark for a standard ROS 2 Nav2 + "
        "slam_toolbox stack under LiDAR noise and dropout, in Gazebo simulation."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    # ground_truth_node lands here once it's actually implemented (see
    # CHECKPOINT.md) - no entry for a module that doesn't exist yet.
    entry_points={
        "console_scripts": [
            "nav_mission_node = nav2_slam_resilience.nav_mission_node:main",
        ],
    },
)
