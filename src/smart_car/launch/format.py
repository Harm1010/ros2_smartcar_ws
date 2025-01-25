import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def create_launch_description():
    # Directories for required packages
    pkg_smartcar_dir = get_package_share_directory('smart_car')
    pkg_gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # File paths for various configurations
    localization_config = os.path.join(pkg_smartcar_dir, 'config', 'ekf.yaml')
    urdf_path = os.path.join(pkg_smartcar_dir, 'urdf', 'smartcar.urdf')
    rviz_config_path = os.path.join(pkg_smartcar_dir, 'config', 'smartcar_nav2.rviz')
    nav2_params_path = os.path.join(pkg_smartcar_dir, 'config', 'nav2_params.yaml')
    behavior_tree_path = os.path.join(pkg_smartcar_dir, 'config', 'bh_tree_nav2.xml')
    world_path = os.path.join(pkg_smartcar_dir, 'world', 'smalltown.world')
    map_yaml_path = os.path.join(pkg_smartcar_dir, 'map', 'smalltown_world.yaml')

    # Launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Enable simulated time'
    )
    arg_enable_slam = DeclareLaunchArgument(
        'slam', default_value='True', description='Run SLAM or not'
    )
    arg_map_yaml = DeclareLaunchArgument(
        'map', default_value=map_yaml_path, description='Path to map YAML file'
    )

    # Define nodes and launch files
    odom_node = Node(
        package='smart_car',
        executable='wheel_odometry_node.py',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch Gazebo with world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn the robot model in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'smartcar', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Joint State Publisher node
    joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Robot State Publisher
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[urdf_path]
    )

    # RViz node for visualization
    rviz_visualizer = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # EKF Node for localization
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[localization_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch the Navigation2 stack with SLAM and behavior tree configuration
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': map_yaml_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path,
            'autostart': 'true',
            'default_bt_xml_filename': behavior_tree_path
        }.items()
    )

    # Build the complete launch description
    return LaunchDescription([
        arg_use_sim_time,
        arg_enable_slam,
        arg_map_yaml,
        gazebo,
        spawn_robot,
        joint_publisher,
        state_publisher,
        rviz_visualizer,
        ekf_localization,
        odom_node,
        nav2_launch
    ])

