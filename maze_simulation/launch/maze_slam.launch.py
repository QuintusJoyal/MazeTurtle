from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'config',
            'mapper_params_online_async.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    nav2_params_file = LaunchConfiguration('nav2_params_file')
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'params',
            'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for navigation2'
    )

    # Include the maze launch file
    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('maze_simulation'),
                'launch',
                'maze.launch.py'
            ])
        ])
    )

    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': slam_params_file
        }.items()
    )

    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params_file
        }.items()
    )

    # Launch RViz2
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        maze_launch,
        slam_launch,
        nav2_launch,
        rviz_node
    ])