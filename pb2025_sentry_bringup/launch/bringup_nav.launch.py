import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ========================
    # Directories
    # ========================
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    navigation_bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    bt_bringup_dir = get_package_share_directory("pb2025_sentry_behavior")

    # ========================
    # Launch configurations
    # ========================
    robot_name = LaunchConfiguration("robot_name")

    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # ========================
    # Environment
    # ========================
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT", "1"
    )

    # ========================
    # Declare arguments
    # ========================
    declare_cmds = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="lf2026_old_sentry",
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="basef1_1116",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
                world,
                TextSubstitution(text=".yaml"),
            ],
        ),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
                world,
                TextSubstitution(text=".pcd"),
            ],
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        ),
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
        ),
    ]

    # ========================
    # Serial (下位机 / 云台)
    # ========================
    start_serial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_bringup_dir,
                "launch",
                "standard_robot_pp_ros2.launch.py",
            )
        ),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "params_file": params_file,
            "use_rviz": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    # ========================
    # Navigation (Nav2 / SLAM)
    # ========================
    start_navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_bringup_dir,
                "launch",
                "rm_navigation_reality_launch.py",
            )
        ),
        launch_arguments={
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_rviz": "False",
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    # ========================
    # Behavior Tree
    # ========================
    start_behavior_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bt_bringup_dir,
                "launch",
                "pb2025_sentry_behavior_launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "log_level": log_level,
        }.items(),
    )

    # ========================
    # RViz（可选）
    # ========================
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(
                bringup_dir, "rviz", "sentry_default_view.rviz"
            ),
        }.items(),
    )

    # ========================
    # Rosbag
    # ========================
    record_rosbag_cmd = Node(
        package="rosbag2_composable_recorder",
        executable="composable_recorder_node",
        name="rosbag_recorder",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ========================
    # LaunchDescription
    # ========================
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    for cmd in declare_cmds:
        ld.add_action(cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(start_serial_driver_cmd)
    ld.add_action(start_navigation_launch_cmd)
    ld.add_action(start_behavior_launch_cmd)
    ld.add_action(record_rosbag_cmd)

    return ld

