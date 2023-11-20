from abc import ABCMeta
from model.world.map_legacy.map import Map
from model.geometry.point import Point


class Controller(metaclass=ABCMeta):

    def __init__(self):

        self.path = []

    def step_motion(self, map: Map):
        pass

    def is_robot_at(self):
        return True

    def get_next_point(self):
        return None

    def compute_path(self):
        pass
