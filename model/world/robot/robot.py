from abc import ABCMeta, abstractmethod
from scipy.spatial import ConvexHull
import numpy as np

from model.geometry.polygon import Polygon
from model.geometry.point import Point


EPS = 0.001  # 1 mm


class Robot(metaclass=ABCMeta):

    def __init__(self, name, bodies, motors=None):

        # Name of the robot
        self.name = name

        # Robot starts at the origin
        self.previous_pose = (0, 0, 0)
        self.current_pose = (0, 0, 0)
        self.target_pose = (0, 0, 0)

        self.speed_multiplier = 1

        # Robot base consists of multiple polygons
        self.bodies = bodies

        # The polygon is the outline of the entire robot. It will
        # serve to check collisions
        points = []
        for body in bodies:
            for point in body.points:
                points.append(point.to_array())

        # Take only the outermost among them
        hull = ConvexHull(points)
        outermost_points = [points[i] for i in hull.vertices]
        self.outline = Polygon(outermost_points)

        # Sensor objects
        self.sensors = []

        # Motor objects
        self.motors = []

    def add_sensor(self, sensor, pose, is_deg=True):

        # The pose is relative to the center of the robot
        sensor.polygon.rotate_around(0, 0, pose[2], is_deg)
        sensor.polygon.translate(pose[0], pose[1])
        self.sensors.append(sensor)

    def step_motion(self, dt):
        """
        Simulate the obstacle's motion over the given time interval
        """

        # Store the current pose
        self.previous_pose = self.current_pose

        current_x, current_y, current_theta = self.current_pose
        target_x, target_y, target_theta = self.target_pose

        distance = np.sqrt(np.power(target_x - current_x, 2) + np.power(target_y - current_y, 2))
        if distance < EPS:

            current_x = target_x
            current_y = target_y

        else:

            delta_x = target_x - current_x
            delta_y = target_y - current_y

            current_x = current_x + delta_x * dt * self.speed_multiplier
            current_y = current_y + delta_y * dt * self.speed_multiplier

        distance_theta = abs(target_theta - current_theta)
        if distance_theta < EPS * 10:

            current_theta = target_theta

        else:

            # This is if we use radians in the pose
            # delta_theta = np.arctan2(np.sin(target_theta - current_theta), np.cos(target_theta - current_theta))
            delta_theta = target_theta - current_theta
            current_theta = current_theta + delta_theta * dt * self.speed_multiplier

        self.current_pose = (current_x, current_y, current_theta)

        # Update the estimated pose
        self.apply_dynamics(dt)

        # Update the geometries
        self.update_geometry()

    def update_geometry(self):

        # Compute the displacement from the previous pose
        current_x, current_y, current_theta = self.current_pose
        previous_x, previous_y, previous_theta = self.previous_pose

        dx = current_x - previous_x
        dy = current_y - previous_y
        dtheta = (current_theta - previous_theta) % 360

        # Update the bodies
        for polygon in self.bodies:
            polygon.translate(dx, dy)
            polygon.rotate_around(current_x, current_y, dtheta)

        # Update the outline polygon
        self.outline.translate(dx, dy)
        self.outline.rotate_around(current_x, current_y, dtheta)

        # Update the sensor polygons
        for sensor in self.sensors:
            sensor.polygon.translate(dx, dy)
            sensor.polygon.rotate_around(current_x, current_y, dtheta)

        # Update the motor polygons
        for motor in self.motors:
            motor.polygon.translate(dx, dy)
            motor.polygon.rotate_around(current_x, current_y, dtheta)

    @abstractmethod
    def apply_dynamics(self, dt):
        return

    @abstractmethod
    def add_motor(self, motor, pose):
        # Number of motors is constrained based on the type of the robot
        return
