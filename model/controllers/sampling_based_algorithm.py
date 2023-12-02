from abc import abstractmethod

from model.controllers.search_algorithm import SearchAlgorithm
from model.geometry.segment import Segment
from model.geometry.point import Point


class SamplingBased(SearchAlgorithm):

    def __init__(self, map, start=Point(0, 0), boundary=0.2, iterations=1):

        self.nodes = []
        self.edges = []

        super().__init__(map, start, boundary, iterations)

    def update_draw_list(self, node):
        child_point = node.point
        parent_point = node.parent.point if node.parent is not None else None
        self.draw_list.append(child_point)
        if parent_point is not None:
            self.draw_list.append(Segment(parent_point, child_point))

    @abstractmethod
    def heuristic(self, point):
        pass


