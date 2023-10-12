import random


class Obstacle:

    def __init__(self, polygon, pose, vx=None, vy=None):

        self.polygon = polygon
        self.pose = pose

        # Speed
        if vx is None:
            self.vx = random.random() * 2 - 1
        else:
            self.vx = vx

        if vy is None:
            self.vy = random.random() * 2 - 1
        else:
            self.vy = vy

    def set_speed(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def step_motion(self, dt):
        """
        Simulate the obstacle's motion over the given time interval
        """

        x = self.pose[0]
        y = self.pose[1]
        self.pose = (x + self.vx * dt, y + self.vy * dt, self.pose[2])

        self.update_geometry()

    def update_geometry(self):
        self.polygon.transform(self.pose)
