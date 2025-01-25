#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smartcar_msgs.msg import Status  
from nav_msgs.msg import Odometry  
from geometry_msgs.msg import Quaternion
import math
import time

class WheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('wheel_odom_publisher')

        # Subscriber to vehicle status topic
        self.subscription = self.create_subscription(
            Status,
            '/smart_car/vehicle_status',
            self.vehicle_status_callback,
            10
        )

        # Publisher for wheel odometry
        self.odom_publisher = self.create_publisher(Odometry, '/smart_car/wheel/odom', 10)

        # car parameters
        self.wheel_radius = 0.032  # Radius in meters
        self.axle_distance = 0.2535 # Distance between front and rear axle in meters
        
        #calculation parameters or constants required
        self.rpm_rad_s_conversion = 0.10472 #The conversion factor for going from rpm to rad/s
        
        #Definition of x location, y location, phi, linear and angular velocity
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.phi_prev = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.steering_angle_rad = 0.0
        
        #Storing time for position calculations 
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        
    def vehicle_status_callback(self, msg:Status):     #function which processes incoming data 
        engine_speed_rpm = msg.engine_speed_rpm
        steering_angle_rad = msg.steering_angle_rad
        
        # Calculating linear velocity (m/s) from engine RPM
        self.linear_velocity = ((engine_speed_rpm * math.pi * self.wheel_radius)/60)
        
        # Calculating angular velocity in [rad/s], 1 RPM = 0,10472 rad/s
        self.angular_velocity = (self.linear_velocity/self.axle_distance)*math.tan(steering_angle_rad)
        
        #Current time and dt calc
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.last_time
        self.update_pose(dt)
        self.last_time = current_time
        
    def update_pose(self, dt):      #function to update the cars pose and position
        #orientation, phi
        self.phi_prev += self.angular_velocity * dt
        #x, y position
        self.x_prev += self.linear_velocity * math.cos(self.phi_prev) * dt
        self.y_prev += self.linear_velocity * math.sin(self.phi_prev) * dt
        #publishing
        self.publish_wheel_odometry(self.linear_velocity, self.angular_velocity)        

    def publish_wheel_odometry(self, linear_velocity, steering_angle_rad):      #publishes wheel odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Set position and orientation
        odom_msg.pose.pose.position.x = self.x_prev
        odom_msg.pose.pose.position.y = self.y_prev
        
        # Define covariance values
        position_variance = 1.0
        orientation_variance = 0.5
        unused_variance = 100000.0
        angular_velocity_variance = 0.1

        # Fill covariance matrix for pose
        odom_msg.pose.covariance = [position_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, position_variance, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, unused_variance, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, unused_variance, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, orientation_variance, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, orientation_variance]

        # Fill covariance matrix for twist (angular velocity)
        odom_msg.twist.covariance = [angular_velocity_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, angular_velocity_variance, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, unused_variance, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, unused_variance, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, angular_velocity_variance, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, angular_velocity_variance]
        
        # Convert the orientation from radians to quaternion
        q = Quaternion()
        q.w = math.cos(self.phi_prev / 2)
        q.z = math.sin(self.phi_prev / 2)
        odom_msg.pose.pose.orientation = q
        
        # Set linear and angular velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = steering_angle_rad
        print(odom_msg)
        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
