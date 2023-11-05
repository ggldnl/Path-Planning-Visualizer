from model.world.robot.robot import Robot
from model.geometry.point import Point

import numpy as np


class DifferentialDriveRobot(Robot):

    def __init__(self, bodies, motors=None):
        super().__init__('Differential Drive Robot', bodies, motors)

        # TODO set this
        self.wheelbase = 0.2

    def apply_dynamics(self, dt):

        current_x, current_y, current_theta = self.current_pose
        target_x, target_y, target_theta = self.target_pose

        # The robot is now at the target, we need to reach the target orientation
        if current_x == target_x and current_y == target_y:

            delta_theta = target_theta - current_theta

            if abs(delta_theta) > self.ROTATION_EPSILON:
                current_theta += self.angular_velocity * dt * np.copysign(1, delta_theta)
            else:
                current_theta = target_theta

        else:

            # Calculate the angle between the current orientation and the target orientation
            delta_x = target_x - current_x
            delta_y = target_y - current_y

            target_angle = np.rad2deg(np.arctan2(delta_y, delta_x))
            delta_theta = target_angle - current_theta

            # Rotate the robot towards the new point
            if abs(delta_theta) > self.ROTATION_EPSILON:
                current_theta += self.angular_velocity * dt * np.copysign(1, delta_theta)

            else:
                # The robot points to the target
                current_theta = target_angle

                # Move the robot forward
                increment_x = self.linear_velocity * dt * np.cos(np.deg2rad(current_theta))
                if target_x >= current_x:
                    if current_x + increment_x >= target_x:
                        current_x = target_x
                    else:
                        current_x += increment_x
                else:
                    if current_x + increment_x >= target_x:
                        current_x += increment_x
                    else:
                        current_x = target_x

                increment_y = self.linear_velocity * dt * np.sin(np.deg2rad(current_theta))
                if target_y >= current_y:
                    if current_y + increment_y >= target_y:
                        current_y = target_y
                    else:
                        current_y += increment_y
                else:
                    if current_y + increment_y >= target_y:
                        current_y += increment_y
                    else:
                        current_y = target_y

                print(f'current {self.current_pose} -> target {self.target_pose}')
                print(f'current_x({current_x} | target_x({target_x}) | current_y({current_y}) | target_y({target_y})')
                print()

        self.current_pose = (current_x, current_y, current_theta)

    def compute_odometry(self, dt):

        # TODO finish implementation and test

        # Compute the change in wheel angle (in radians)
        d_angle_left = dt * self.motors[0].speed
        d_angle_right = dt * self.motors[1].speed

        # Compute the distance traveled
        right_wheel_meters_per_rad = self.motors[0].wheel_radius
        d_right_wheel = d_angle_right * right_wheel_meters_per_rad

        left_wheel_meters_per_rad = self.motors[1].wheel_radius
        d_left_wheel = d_angle_left * left_wheel_meters_per_rad

        d_center = (d_left_wheel + d_right_wheel) / 2.0

        # Compute the new pose (estimated)
        old_x, old_y, old_theta = self.estimated_pose
        new_x = old_x + (d_center * np.cos(old_theta))
        new_y = old_y + (d_center * np.sin(old_theta))
        new_theta = old_theta + (
            (d_right_wheel - d_left_wheel) / self.wheelbase
        )

        # Compute the number of rotations each wheel has made
        revolutions_left = d_angle_left / (2 * np.pi)
        revolutions_right = d_angle_right / (2 * np.pi)

        # Update the state of the moving parts
        self.estimated_pose = (new_x, new_y, new_theta)
        self.motors[0].encoder.step_revolutions(revolutions_left)
        self.motors[1].encoder.step_revolutions(revolutions_right)

    def add_motor(self, motor, pose):
        if len(self.motors) == 2:
            raise ValueError('Differential drive robots only have two motors')
