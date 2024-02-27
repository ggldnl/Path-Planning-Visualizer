from model.world.robot.robot import Robot
from model.geometry.pose import Pose

import numpy as np


class DifferentialDriveRobot(Robot):

    def __init__(self, bodies, wheelbase):
        super().__init__('Differential Drive Robot', bodies)
        self.wheelbase = wheelbase

    def apply_dynamics(self, dt):

        current_x, current_y, current_theta = self.current_pose
        target_x, target_y, target_theta = self.target_pose

        # The robot is now at the target, we need to reach the target orientation
        if current_x == target_x and current_y == target_y:

            # Calculate the difference between current and target theta
            theta_diff = target_theta - current_theta

            # Normalize the difference to the range [-pi, pi)
            theta_diff = (theta_diff + np.pi) % (2 * np.pi) - np.pi

            # Determine the direction of rotation (+1 for clockwise, -1 for counterclockwise)
            direction = 1 if theta_diff > 0 else -1

            # Calculate the incremental rotation step
            step = self.angular_velocity * dt * direction

            # Update the current theta
            current_theta = (current_theta + step) % (2 * np.pi)

            # Adjust theta to the correct range
            current_theta = (current_theta + 2 * np.pi) % (2 * np.pi)

            # Check if the rotation is close enough to the target angle
            if abs(theta_diff) < self.ROTATION_EPSILON:
                current_theta = target_theta

        else:

            # Calculate the angle between the current orientation and the target orientation
            delta_x = target_x - current_x
            delta_y = target_y - current_y

            target_angle = np.arctan2(delta_y, delta_x)

            # Calculate the difference between current and target theta
            theta_diff = target_angle - current_theta

            # Normalize the difference to the range [-pi, pi)
            theta_diff = (theta_diff + np.pi) % (2 * np.pi) - np.pi

            # Determine the direction of rotation (+1 for clockwise, -1 for counterclockwise)
            direction = 1 if theta_diff > 0 else -1

            # Calculate the incremental rotation step
            step = self.angular_velocity * dt * direction

            # Update the current theta
            current_theta = (current_theta + step) % (2 * np.pi)

            # Adjust theta to the correct range
            current_theta = (current_theta + 2 * np.pi) % (2 * np.pi)

            # Check if the rotation is close enough to the target angle
            if abs(theta_diff) < self.ROTATION_EPSILON:

                # The robot points to the target
                current_theta = target_angle

                # Move the robot forward
                increment_x = self.linear_velocity * dt * np.cos(current_theta)
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

                increment_y = self.linear_velocity * dt * np.sin(current_theta)
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

        self.current_pose = Pose(current_x, current_y, current_theta)

    def compute_odometry(self, dt):

        # TODO finish implementation and test
        """
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
        """
