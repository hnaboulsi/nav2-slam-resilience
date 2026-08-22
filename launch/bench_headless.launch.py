"""Headless benchmark launch: Gazebo (server-only) + TurtleBot3 + slam_toolbox
(sync) + Nav2 (navigating-while-mapping mode), with the LiDAR fault path
wired in between the simulator and the SLAM/Nav2 stack.

Everything here is a standard package's own launch file, included rather
than reimplemented: `ros_gz_sim`'s `gz_sim.launch.py`, `turtlebot3_gazebo`'s
`robot_state_publisher.launch.py`, `slam_toolbox`'s `online_sync_launch.py`,
and `nav2_bringup`'s `bringup_launch.py` (included directly, not through
`turtlebot3_navigation2`'s wrapper, because that wrapper hardcodes
`use_sim_time:=false` and doesn't expose `slam:=`). The only two things this
file adds are: (1) a validated text patch of the stock LiDAR model's noise
stddev at spawn time (`nav2_slam_resilience.model_patch`, unit-tested against
the real vendor file - see tests/unit/test_model_patch.py), and (2) a
`topic_tools throttle` node sitting on the real LiDAR topic before anything
else sees it, so both a noise sweep (baked into the spawned model) and a
dropout/rate sweep (the throttle rate) share one fault path: Gazebo's real
sensor always publishes to `/scan_raw`, unmodified; what slam_toolbox and
Nav2 actually consume is `/scan`, the throttled copy - so `/scan_raw` stays
available the whole time as an independent check on what was really applied.
"""

from __future__ import annotations

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_slam_resilience.model_patch import patch_lidar_noise_stddev

TURTLEBOT3_MODEL = os.environ.get("TURTLEBOT3_MODEL", "burger")


def _spawn_patched_robot(context, *args, **kwargs):
    """Patch the stock model's LiDAR noise stddev, then spawn it.

    Done as an OpaqueFunction (not a plain Node action) because the noise
    value is only known at launch time (a LaunchConfiguration), but the
    patched SDF has to exist as a real file on disk before `ros_gz_sim
    create` can point `-file` at it.
    """
    noise_stddev = float(LaunchConfiguration("noise_stddev").perform(context))
    x_pose = LaunchConfiguration("x_pose").perform(context)
    y_pose = LaunchConfiguration("y_pose").perform(context)

    model_folder = f"turtlebot3_{TURTLEBOT3_MODEL}"
    stock_model_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        model_folder,
        "model.sdf",
    )
    with open(stock_model_path) as f:
        stock_sdf = f.read()
    patched_sdf = patch_lidar_noise_stddev(stock_sdf, noise_stddev)

    tmp_dir = tempfile.mkdtemp(prefix="nav2_slam_resilience_")
    patched_model_path = os.path.join(tmp_dir, "model.sdf")
    with open(patched_model_path, "w") as f:
        f.write(patched_sdf)

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", TURTLEBOT3_MODEL,
            "-file", patched_model_path,
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.01",
        ],
        output="screen",
    )
    return [spawn_node]


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    turtlebot3_navigation2_dir = get_package_share_directory("turtlebot3_navigation2")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    noise_stddev = LaunchConfiguration("noise_stddev")
    throttle_rate_hz = LaunchConfiguration("throttle_rate_hz")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo's simulation clock"
    )
    declare_noise_stddev_cmd = DeclareLaunchArgument(
        "noise_stddev",
        default_value="0.01",
        description="LiDAR range-noise stddev (m) baked into the spawned model. "
        "0.01 is the stock TurtleBot3 value (no-fault baseline).",
    )
    declare_throttle_rate_cmd = DeclareLaunchArgument(
        "throttle_rate_hz",
        default_value="30.0",
        description="Rate /scan is throttled to before slam_toolbox/Nav2 see it. "
        "30.0 is faster than the LiDAR's native 5 Hz, i.e. a no-op pass-through.",
    )
    declare_x_pose_cmd = DeclareLaunchArgument("x_pose", default_value="-2.0")
    declare_y_pose_cmd = DeclareLaunchArgument("y_pose", default_value="-0.5")

    world = os.path.join(turtlebot3_gazebo_dir, "worlds", "turtlebot3_world.world")

    gz_sim_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items(),
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(turtlebot3_gazebo_dir, "models")
    )

    spawn_robot_cmd = OpaqueFunction(function=_spawn_patched_robot)

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    bridge_params = os.path.join(
        turtlebot3_gazebo_dir, "params", f"turtlebot3_{TURTLEBOT3_MODEL}_bridge.yaml"
    )
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        # Gazebo's real, unmodified LiDAR always lands on /scan_raw - the
        # fault path (throttle, below) decides what /scan actually is.
        remappings=[("scan", "scan_raw")],
        output="screen",
    )

    throttle_cmd = Node(
        package="topic_tools",
        executable="throttle",
        arguments=["messages"],
        parameters=[
            {
                "input_topic": "/scan_raw",
                "output_topic": "/scan",
                "msgs_per_sec": throttle_rate_hz,
            }
        ],
        output="screen",
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"), "launch", "online_sync_launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Included directly from nav2_bringup (not through turtlebot3_navigation2's
    # wrapper) so slam:=true and use_sim_time:=true are actually reachable.
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")),
        launch_arguments={
            # nav2_bringup's bringup_launch.py raw-evals `slam` as Python
            # (PythonExpression(['not ', slam, ' and ', use_localization])),
            # so this must be the capitalized "True"/"False" its own
            # declared default uses, not the lowercase "true" every other
            # launch arg here accepts.
            "slam": "True",
            "use_sim_time": use_sim_time,
            "map": os.path.join(turtlebot3_navigation2_dir, "map", "map.yaml"),
            "params_file": os.path.join(
                turtlebot3_navigation2_dir, "param", f"{TURTLEBOT3_MODEL}.yaml"
            ),
            "autostart": "true",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_noise_stddev_cmd)
    ld.add_action(declare_throttle_rate_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gz_sim_server_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(throttle_cmd)
    ld.add_action(slam_toolbox_cmd)
    ld.add_action(nav2_bringup_cmd)
    return ld
