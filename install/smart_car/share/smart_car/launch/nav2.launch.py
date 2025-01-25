import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define package and file paths
    package_name = 'smart_car'
    gazebo_ros_pkg = 'gazebo_ros'
    nav2_pkg = 'nav2_bringup'
    
    # URDF and world file paths
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'smartcar.urdf')
    world_path = os.path.join(get_package_share_directory(package_name), 'world', 'smalltown.world')
    
    # Configurations for Extended Kalman Filter, Nav2, and RViz
    ekf_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    nav2_map_path = os.path.join(get_package_share_directory(package_name), 'config', 'smalltown_world.yaml')
    nav2_bt_path = os.path.join(get_package_share_directory(package_name), 'config', 'bt_nav2.xml')
    rviz_config_path = '/home/ros2/ros2_ws/src/smart_car/rviz/nav2_default_view.rviz'

    # Gazebo and Nav2 launch files
    gazebo_launch_file = os.path.join(get_package_share_directory(gazebo_ros_pkg), 'launch', 'gazebo.launch.py')
    nav2_launch_file = os.path.join(get_package_share_directory(nav2_pkg), 'launch', 'bringup_launch.py')

    # Declare launch arguments for simulation time and SLAM usage
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    enable_slam = DeclareLaunchArgument('slam', default_value='False')

    # Gazebo Launch
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': world_path}.items()
    )

    # Robot Entity Spawning in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_smart_car',
        output='screen',
        arguments=['-entity', 'smart_car', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0']
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Odometry Publisher Node
    odometry_publisher = Node(
        package='smart_car',
        executable='odom.py',
        name='odometry_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # EKF Node for Localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[("odometry/filtered", "/odom")]
    )

    # Nav2 Launch for Navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path,
            'map': nav2_map_path,
            'auto_start': 'true',
            'bt_xml_file': nav2_bt_path
        }.items()
    )

    # Assemble and return launch description
    return LaunchDescription([
        use_sim_time,
        enable_slam,
        start_gazebo,
        spawn_robot,
        robot_state_publisher,
        rviz_node,
        odometry_publisher,
        ekf_node,
        nav2_launch
    ])

