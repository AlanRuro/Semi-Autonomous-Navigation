import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
        get_package_share_directory("reto-ciberfisicos"), "config", "explore.yaml"),
        description='Full path to the ROS2 parameters file to use for explore node'
    )

    node = Node(
        package="reto-ciberfisicos",
        name="explore_node",
        executable="explore",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_argument,
            declare_namespace_argument,
            declare_log_level_cmd,
            declare_params_file_cmd,
            node,
        ]
    )