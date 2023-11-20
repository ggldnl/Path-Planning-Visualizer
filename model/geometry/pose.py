from model.geometry.point import Point

import numpy as np


class Pose:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def set_deg_theta(self, new_theta):
        self.theta = np.deg2rad(new_theta % 360)

    def __getitem__(self, item):
        if item == 0:
            return self.x
        elif item == 1:
            return self.y
        elif item == 2:
            return self.theta
        raise IndexError(f'Pose index out of range: {item}')

    def __setitem__(self, key, value):
        if key == 0:
            self.x = value
        elif key == 1:
            self.y = value
        elif key == 2:
            self.theta = value
        raise IndexError(f'Pose index out of range: {key}')

    def __iter__(self):
        return iter([self.x, self.y, self.theta])

    def copy(self):
        return Pose(self.x, self.y, self.theta)

    def distance(self, other):
        if isinstance(other, Point) or isinstance(other, Pose):
            return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
        raise ValueError(f'Unsupported distance operation between Pose (self) and {type(other)} (other)')


if __name__ == '__main__':

    pose = Pose(0, 0, 0)
    x, y, theta = pose
    print(f"x: {x}, y: {y}, theta: {theta}")
