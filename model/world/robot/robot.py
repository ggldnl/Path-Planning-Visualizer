from abc import ABCMeta, abstractmethod
from scipy.spatial import ConvexHull

from model.geometry.polygon import Polygon


class Robot(metaclass=ABCMeta):

    def __init__(self, name, bodies, motors=None):

        # Name of the robot
        self.name = name

        # Robot starts at the origin
        self.pose = (0, 0, 0)
        self.estimated_pose = (0, 0, 0)

        # TODO remove this, let the controller set the velocity vector
        self.vel = (0.5, 0.5, 0)
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
        self.body = Polygon(outermost_points)

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

        # Update the real pose
        x, y, z = self.pose
        vx, vy, vz = self.vel
        lsm = self.speed_multiplier
        self.pose = (
            x + vx * lsm * dt,
            y + vy * lsm * dt,
            (z + vz * dt) % 360
        )

        # Update the estimated pose
        self.apply_dynamics(dt)

        # Update the geometries
        self.update_geometry()

    def update_geometry(self):

        # Update the bodies
        for polygon in self.bodies:
            polygon.transform_to(self.pose)

        # Update the polygon
        #self.polygon.transform_to(self.pose)

        # Update the sensor polygons
        for sensor in self.sensors:
            sensor.polygon.transform_to(self.pose)

        # Update the motor polygons
        for motor in self.motors:
            motor.polygon.transform_to(self.pose)

    @abstractmethod
    def apply_dynamics(self, pose):
        return

    @abstractmethod
    def add_motor(self, motor, pose):
        # Number of motors is constrained based on the type of the robot
        return
