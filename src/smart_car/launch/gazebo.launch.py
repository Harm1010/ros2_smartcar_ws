import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to important files
    urdf_path = os.path.join(
        get_package_share_directory('smart_car'),
        'urdf',
        'smartcar.urdf'
    )
    
    world_path = os.path.join(
        get_package_share_directory('smart_car'),
        'world',
        'smalltown.world'
    )

    # Gazebo launch file path from gazebo_ros package
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Launch Description Components
    # 1. Start Gazebo and load the specified world file
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': world_path}.items()
    )
    
    # 2. Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_smartcar_entity',
        output='screen',
        arguments=['-entity', 'smart_car', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0']
    )

    # 3. Robot State Publisher for publishing the robot description and transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )
    
    # 4. RViz visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_visualizer',
        output='screen',
        arguments=['-d', '/home/ros2_smartcar_ws/src/smart_car/rviz/smart_car.rviz']
    )

    # 5. Odometry Publisher for robot's wheel odometry
    odometry_publisher = Node(
        package='smart_car',
        executable='odom.py',
        name='odometry_publisher',
        output='screen'
    )

    # Return Launch Description
    return LaunchDescription([
        start_gazebo,                # Start Gazebo with the specified world file
        spawn_robot,                 # Spawn the robot in Gazebo
        robot_state_publisher,       # Publish robot description and transforms
        rviz_node,                   # Start RViz for visualization
        odometry_publisher           # Start the odometry node
    ])

