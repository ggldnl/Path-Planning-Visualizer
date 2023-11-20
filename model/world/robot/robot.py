from abc import ABCMeta, abstractmethod
from scipy.spatial import ConvexHull
import numpy as np

from model.geometry.polygon import Polygon
from model.geometry.pose import Pose


class Robot(metaclass=ABCMeta):

    def __init__(self, name, bodies, motors=None):

        # Name of the robot
        self.name = name

        # Robot starts at the origin
        self.previous_pose = Pose(0, 0, 0)  # theta = 0 -> robot headed on positive x
        self.current_pose = Pose(0, 0, 0)
        self.target_pose = Pose(0, 0, 0)

        # Odometry
        self.estimated_pose = Pose(0, 0, 0)

        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 0.005  # rad/s

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

        # Define tolerance in translation/rotation
        self.ROTATION_EPSILON = 5  # 5 deg

    def add_sensor(self, sensor, pose):

        # The pose is relative to the center of the robot
        # sensor.polygon.rotate_around(0, 0, pose.theta)
        # sensor.polygon.translate(pose.x, pose.y)
        sensor.polygon.transform(pose)
        self.sensors.append(sensor)

    def step_motion(self, dt):
        """
        Simulate the obstacle's motion over the given time interval
        """

        # Store the current pose
        self.previous_pose = self.current_pose

        # Update the current pose
        self.apply_dynamics(dt)

        # Update the geometries
        self.update_geometry()

    def update_geometry(self):

        # Compute the displacement from the previous pose
        current_x, current_y, current_theta = self.current_pose
        previous_x, previous_y, previous_theta = self.previous_pose

        dx = current_x - previous_x
        dy = current_y - previous_y
        dtheta = (current_theta - previous_theta) % (2 * np.pi)

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
    def compute_odometry(self, dt):
        return

    @abstractmethod
    def add_motor(self, motor, pose):
        # Number of motors is constrained based on the type of the robot
        return

    def is_at_target(self):
        return self.current_pose == self.target_pose
