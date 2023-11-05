import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import random


class Robot:

    def __init__(self, linear_velocity=1.0, angular_velocity=30.0, ROTATION_EPSILON=1.0, TRANSLATION_EPSILON=0.1):
    
        self.current_pose = (0, 0, 0)
        self.target_pose = (2, 2, 90)  # Change this to your desired target
        
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.ROTATION_EPSILON = ROTATION_EPSILON
        self.TRANSLATION_EPSILON = TRANSLATION_EPSILON


    def step_motion(self, dt):

        current_x, current_y, current_theta = self.current_pose
        target_x, target_y, target_theta = self.target_pose

        # Calculate the angle between the current orientation and the target orientation
        delta_x = target_x - current_x
        delta_y = target_y - current_y

        target_angle = np.rad2deg(np.arctan2(delta_y, delta_x))
        delta_theta = target_angle - current_theta

        print(f'[({self.current_pose[0]:2.5f}, {self.current_pose[1]:2.5f}, {self.current_pose[2]:2.5f})] -> [({self.target_pose[0]}, {self.target_pose[1]}, {target_angle:2.5f})]')

        # Check if we need to rotate the robot
        if abs(delta_theta) > self.ROTATION_EPSILON:

            # Rotate the robot around its center
            current_theta = (current_theta + self.angular_velocity * dt * np.copysign(1, delta_theta))

        else:

            current_theta = target_angle

            # Move the robot forward or backward
            delta_x = target_x - current_x
            delta_y = target_y - current_y

            if abs(delta_x) > self.TRANSLATION_EPSILON:
                # Move the robot forward
                current_x += self.linear_velocity * dt * np.cos(np.deg2rad(current_theta))
            else:
                # The robot has reached the target
                current_x = target_x

            if abs(delta_y) > self.TRANSLATION_EPSILON:
                current_y += self.linear_velocity * dt * np.sin(np.deg2rad(current_theta))
            else:
                current_y = target_y

            # self.current_pose = (current_x, current_y, current_theta)

        self.current_pose = (current_x, current_y, current_theta)

    def is_at_target(self):
        # return self.current_pose == self.target_pose
        return self.current_pose[0] == self.target_pose[0] and self.current_pose[1] == self.target_pose[1]

# Create a Robot instance
robot = Robot()

# Create a function to update the plot in each animation frame
def update(frame):
    
    robot.step_motion(0.05)
    if robot.is_at_target():
        robot.target_pose = (random.randint(-2, 2), random.randint(-2, 2), 90)

    x, y, theta = robot.current_pose
    half_width, half_height = 0.5, 0.5  # Half of the rectangle's width and height
    x1 = x - half_width * np.cos(np.deg2rad(theta)) - half_height * np.sin(np.deg2rad(theta))
    y1 = y - half_width * np.sin(np.deg2rad(theta)) + half_height * np.cos(np.deg2rad(theta))
    x2 = x + half_width * np.cos(np.deg2rad(theta)) - half_height * np.sin(np.deg2rad(theta))
    y2 = y + half_width * np.sin(np.deg2rad(theta)) + half_height * np.cos(np.deg2rad(theta))
    x3 = x + half_width * np.cos(np.deg2rad(theta)) + half_height * np.sin(np.deg2rad(theta))
    y3 = y + half_width * np.sin(np.deg2rad(theta)) - half_height * np.cos(np.deg2rad(theta))
    x4 = x - half_width * np.cos(np.deg2rad(theta)) + half_height * np.sin(np.deg2rad(theta))
    y4 = y - half_width * np.sin(np.deg2rad(theta)) - half_height * np.cos(np.deg2rad(theta))

    robot_rect.set_xy(np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]]))  # Set the rectangle's corners

    target_point.set_data(robot.target_pose[0], robot.target_pose[1])

    return robot_rect, target_point

# Set up the plot
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)  # Adjust the limits according to your needs
ax.set_ylim(-5, 5)
robot_rect = patches.Polygon([[0, 0]], closed=True, fc='b', alpha=0.5)
ax.add_patch(robot_rect)

target_point, = ax.plot([], [], 'ro', markersize=5)  # Red circle marker

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=50, blit=True)

# Show the plot
plt.show()
