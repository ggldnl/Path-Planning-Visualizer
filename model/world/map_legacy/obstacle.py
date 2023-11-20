from model.geometry.polygon import Polygon
from model.geometry.point import Point
import random


class Obstacle:

    def __init__(self, polygon, pose, vel=None):

        self.polygon = polygon
        self.pose = pose

        if vel is None:
            self.vel = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        else:
            self.vel = vel

        # Global speed multipliers (the obstacles moves proportionally to these values)
        self.linear_speed_multiplier = 1
        self.angular_speed_multiplier = 1

    def step_motion(self, dt):
        """
        Simulate the obstacle's motion over the given time interval
        """

        # Update the pose based on the motion law (translation + rotation)
        x, y, z = self.pose
        vx, vy, vz = self.vel
        lsm = self.linear_speed_multiplier
        asm = self.angular_speed_multiplier
        self.pose = (
            x + vx * lsm * dt,
            y + vy * lsm * dt,
            (z + vz * asm * dt) % 360
        )

        self.update_geometry()

    def update_geometry(self):
        # self.polygon.translate_to(self.pose[0], self.pose[1])
        self.polygon.transform_to(self.pose)

    def copy(self):
        new_polygon = self.polygon.copy()
        new_pose = (self.pose[0], self.pose[1], self.pose[2])
        new_vel = (self.vel[0], self.vel[1], self.vel[2])
        return Obstacle(new_polygon, new_pose, new_vel)

    def to_dict(self):
        return {'polygon': self.polygon.to_dict(), 'pose': self.pose, 'vel': self.vel}

    @classmethod
    def from_dict(cls, dictionary):
        polygon = Polygon.from_dict(dictionary['polygon'])
        pose = tuple(dictionary['pose'])
        vel = tuple(dictionary['vel'])
        return Obstacle(polygon, pose, vel)


class RectangularObstacle(Obstacle):

    def __init__(self, width, height, pose, vel=None):

        if width <= 0:
            raise ValueError(f'Invalid width: {width}')

        if height <= 0:
            raise ValueError(f'Invalid height: {height}')

        polygon = Polygon([
            Point(0, 0),
            Point(0, height),
            Point(width, height),
            Point(width, 0)
        ])

        super().__init__(polygon, pose, vel)
