from model.geometry.pose import Pose
from abc import abstractmethod


class Shape:

    def __init__(self):

        self.pose = Pose(0, 0, 0)
        self.radius = 0
