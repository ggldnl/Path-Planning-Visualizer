from model.world.robot.robot import Robot

import numpy as np


class NonHolonomicRobot(Robot):

    def __init__(self, bodies, motors=None):
        super().__init__('Non Holonomic Robot', bodies, motors)

    def apply_dynamics(self, dt):

        current_x, current_y, current_theta = self.current_pose
        target_x, target_y, target_theta = self.target_pose

        distance = np.sqrt(np.power(target_x - current_x, 2) + np.power(target_y - current_y, 2))
        if distance < self.TRANSLATION_EPSILON:

            current_x = target_x
            current_y = target_y

        else:

            delta_x = target_x - current_x
            delta_y = target_y - current_y

            current_x = current_x + delta_x * dt * self.linear_velocity
            current_y = current_y + delta_y * dt * self.linear_velocity

        distance_theta = abs(target_theta - current_theta)
        if distance_theta < self.ROTATION_EPSILON:

            current_theta = target_theta

        else:

            # This is if we use radians in the pose
            # delta_theta = np.arctan2(np.sin(target_theta - current_theta), np.cos(target_theta - current_theta))
            delta_theta = target_theta - current_theta
            current_theta = current_theta + delta_theta * dt * self.angular_velocity

        self.current_pose = (current_x, current_y, current_theta)

    def compute_odometry(self, dt):
        return

    def add_motor(self, motor, pose):
        if len(self.motors) == 2:
            raise ValueError('Differential drive robots only have two motors')
