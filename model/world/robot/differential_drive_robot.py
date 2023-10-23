from model.world.robot.robot import Robot

import numpy as np


class DifferentialDriveRobot(Robot):

    def __init__(self, bodies, motors=None):
        super().__init__('Differential Drive Robot', bodies, motors)

        self.wheel_base_length = None

    def apply_dynamics(self, dt):

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
            (d_right_wheel - d_left_wheel) / self.wheel_base_length
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
