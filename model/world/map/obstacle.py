import model.world.map.motion_laws as motion_laws
from model.geometry.polygon import Polygon
from model.geometry.point import Point
import random


class Obstacle:

    def __init__(self, polygon, pose, vel=None, motion_law=None):

        self.polygon = polygon
        self.pose = pose

        if vel is None:
            self.vel = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0, 1))
        else:
            self.vel = vel

        if motion_law is None:
            self._motion_law_function = motion_laws.translation_and_rotation_motion
        else:
            self._motion_law_function = motion_law

    def step_motion(self, dt):
        """
        Simulate the obstacle's motion over the given time interval
        """

        # x = self.pose[0]
        # y = self.pose[1]
        # self.pose = (x + self.vx * dt, y + self.vy * dt, self.pose[2])

        self.pose = self._motion_law_function(self.pose, self.vel, dt)

        self.update_geometry()

    def update_geometry(self):
        # self.polygon.translate_to(self.pose[0], self.pose[1])
        self.polygon.transform_to(self.pose)


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
