from model.geometry.pose import Pose
from abc import abstractmethod


class Shape:

    def __init__(self):

        self.pose = Pose(0, 0, 0)
        self.radius = 0

    @abstractmethod
    def get_bounds(self):
        pass

    @abstractmethod
    def translate(self, x, y):
        pass

    @abstractmethod
    def rotate(self, theta):
        pass

    @abstractmethod
    def transform(self, x, y, theta):
        pass

    @abstractmethod
    def translate_to(self, x, y):
        pass

    @abstractmethod
    def rotate_to(self, theta):
        pass

    @abstractmethod
    def transform_to(self, x, y, theta):
        pass

    @abstractmethod
    def rotate_around(self, x, y, theta):
        pass

    @abstractmethod
    def copy(self):
        pass

    @abstractmethod
    def to_dict(self):
        pass

    @classmethod
    def from_dict(cls, shape_dict):
        pass

    @abstractmethod
    def project(self, axis):
        pass

    @abstractmethod
    def get_edges(self):
        pass

    def check_nearness(self, other):
        return self.pose.distance(other.pose) <= self.radius + other.radius
