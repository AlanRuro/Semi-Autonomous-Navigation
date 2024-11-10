from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition



def generate_launch_description():
    reto_dir = FindPackageShare('reto-ciberfisicos')

    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')


    # Declare the launch arguments 

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([reto_dir, 'maps', 'mapab.yaml']),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([reto_dir, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([reto_dir, 'config', 'slam.yaml']),
        description='Full path to the ROS2 parameters file to use for slam node'
    )
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([reto_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for navigation nodes'
    )
    
    declare_amcl_params_file_cmd = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=PathJoinSubstitution([reto_dir, 'config', 'amcl.yaml']),
        description='Full path to the ROS2 parameters file to use for amcl nodes'
    )
    
    declare_explore_params_file_cmd = DeclareLaunchArgument(
        'explore_params_file',
        default_value=PathJoinSubstitution([reto_dir, 'config', 'explore.yaml']),
        description='Full path to the ROS2 parameters file to use for explore node'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            reto_dir, 'rviz', 'explore_rviz.rviz']),
        description='Full path to the RVIZ config file to use'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'
    )


    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reto_dir, 'launch', 'rviz.launch.py'])),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'config_file': rviz_config_file}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reto_dir, 'launch', 'slam.launch.py'])
        ),
        condition=IfCondition(slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': slam_params_file
        }.items(),
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reto_dir, 'launch', 'amcl.launch.py'])
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': amcl_params_file,
            'map': map_yaml_file,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reto_dir, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'log_level': log_level,
        }.items()
    )

    explore_lite_launch =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reto_dir, 'launch', 'explore.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': explore_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_slam_cmd,
            declare_use_sim_time_cmd,
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_slam_params_file_cmd,
            declare_nav2_params_file_cmd,
            declare_amcl_params_file_cmd,
            declare_explore_params_file_cmd,
            declare_rviz_config_file_cmd,
            declare_use_rviz_cmd,
            declare_log_level_cmd,
            rviz_launch,
            slam_launch,
            amcl_launch,
            nav2_launch,
            # explore_lite_launch,
        ]
    )
