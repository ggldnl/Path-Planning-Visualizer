import model.world.map.motion_laws as motion_laws

import random


class Obstacle:

    def __init__(self, polygon, pose, vel=None):

        self.polygon = polygon
        self.pose = pose

        if vel is None:
            self.vel = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0, 1))
        else:
            self.vel = vel

        self._motion_law_function = motion_laws.bounded_window_motion

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
        self.polygon.transform_to(self.pose)

