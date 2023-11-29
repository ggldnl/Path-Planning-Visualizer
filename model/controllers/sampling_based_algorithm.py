from abc import abstractmethod

from model.controllers.search_algorithm import SearchAlgorithm
from model.geometry.segment import Segment
from model.geometry.point import Point


class SamplingBased(SearchAlgorithm):

    def __init__(self, map, start=Point(0, 0), boundary=0.2):

        self.nodes = []
        self.edges = []

        super().__init__(map, start, boundary)

    def get_view(self, parent_point, child_point):
        return Segment(parent_point, child_point), child_point

    @abstractmethod
    def heuristic(self, point):
        pass


