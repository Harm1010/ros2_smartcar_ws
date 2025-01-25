import math
import time
from collections import namedtuple

# Simple replacement for geometry messages
TransformStamped = namedtuple("TransformStamped", ["header", "transform"])
Quaternion = namedtuple("Quaternion", ["x", "y", "z", "w"])

class JointStatePublisher:
    def __init__(self):
        # Set initial values
        self.steering_angle_rad = 0.0
        self.engine_speed_rpm = 0.0
        self.wheel_radius = 0.032  # Radius of the wheel in meters
        self.wheel_base = 0.2535   # Distance between front and back wheels in meters

        self.rate = 0.1  # Publish every 0.1 seconds

    def steering_angle_callback(self, steering_angle_rad):
        """Manually set the steering angle in radians"""
        self.steering_angle_rad = steering_angle_rad

    def engine_speed_callback(self, engine_speed_rpm):
        """Manually set the engine speed in RPM"""
        self.engine_speed_rpm = engine_speed_rpm

    def publish_joint_states(self):
        """Calculate the joint states (wheel speeds)"""
        # Angular velocity in rad/s from RPM
        angular_velocity = self.engine_speed_rpm * (2 * math.pi / 60)
        wheel_rotation_speed = angular_velocity / self.wheel_radius

        # Simulated joint state for four wheels
        joint_state = {
            'front_left_wheel': wheel_rotation_speed,
            'front_right_wheel': wheel_rotation_speed,
            'back_left_wheel': wheel_rotation_speed,
            'back_right_wheel': wheel_rotation_speed
        }

        print("Joint states published:", joint_state)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to a quaternion (used in 3D rotations)"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(qx, qy, qz, qw)

    def publish_transform(self):
        """Calculate the transformation for the front wheels"""
        # Calculate translation based on the steering angle
        delta_x = self.wheel_base * math.cos(self.steering_angle_rad)
        delta_y = self.wheel_base * math.sin(self.steering_angle_rad)
        
        # Simulate the transformation message for front wheels
        transform = TransformStamped(
            header={"frame_id": "base_link", "child_frame_id": "front_wheels"},
            transform={
                "translation": {"x": delta_x, "y": delta_y, "z": 0.0},
                "rotation": self.quaternion_from_euler(0, 0, self.steering_angle_rad)
            }
        )

        print("Transform broadcasted:", transform)

    def run(self):
        """Run the node simulation"""
        try:
            while True:
                self.publish_joint_states()
                self.publish_transform()
                time.sleep(self.rate)
        except KeyboardInterrupt:
            print("Node stopped")

# Usage example
if __name__ == '__main__':
    node = JointStatePublisher()

    # Manually simulate setting values and running the loop
    node.steering_angle_callback(0.1)  # Simulate steering angle of 0.1 rad
    node.engine_speed_callback(1000)   # Simulate engine speed of 1000 RPM

    node.run()
