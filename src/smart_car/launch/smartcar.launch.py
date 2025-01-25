import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths for URDF and RViz2 configuration
    package_name = 'smart_car'
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'smartcar.urdf')
    rviz2_config_path = os.path.join(get_package_share_directory(package_name), 'launch', 'smartcar.launch.py')

    return LaunchDescription([
        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),
        
        # Joint State Publisher GUI Node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz2 Visualization Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'rviz2_config_file': rviz2_config_path
            }]
        ),
        
        # Extended Kalman Filter (EKF) Node for Localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Set to true if using simulation time
                'odom0': '/odom',  # Odometry topic
                'imu0': '/imu/data',  # IMU data topic
                'imu0_config': [
                    True, True, True,  # Orientation (x, y, z)
                    False, False, False,  # Unused orientation rates
                    True, True, True,  # Angular velocity (x, y, z)
                    True, True, True  # Linear acceleration (x, y, z)
                ],
                'imu0_differential': False,
                'imu0_remove_gravitational_acceleration': True
            }]
        )
    ])

