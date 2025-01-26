import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for the Gazebo simulation.

    This function sets up the necessary nodes and includes launch descriptions
    for launching a smart car simulation in Gazebo. It includes spawning the 
    robot model, publishing its state, and visualizing it in RViz.
    """

    # Path to the smart car URDF file
    urdf_path = os.path.join(
        get_package_share_directory('smart_car'),
        'urdf',
        'smartcar.urdf'
    )
    
    # Path to the Gazebo world file
    world_path = os.path.join(
        get_package_share_directory('smart_car'),
        'world',
        'smalltown.world'
    )

    # Path to the Gazebo launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Include the Gazebo launch file with the specified world
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': world_path}.items()
    )
    
    # Node to spawn the smart car entity in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_smartcar_entity',
        output='screen',
        arguments=['-entity', 'smart_car', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0']
    )

    # Node to publish the robot's state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    # Node to visualize the robot in RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_visualizer',
        output='screen',
        arguments=['-d', '/home/ros2_smartcar_ws/src/smart_car/rviz/smart_car.rviz']
    )

    # Node to publish odometry data
    odometry_publisher = Node(
        package='smart_car',
        executable='odom.py',
        name='odometry_publisher',
        output='screen'
    )

    # Return the launch description with all the nodes and included launch files
    return LaunchDescription([
        start_gazebo,                
        spawn_robot,                 
        robot_state_publisher,       
        rviz_node,                   
        odometry_publisher           
    ])

