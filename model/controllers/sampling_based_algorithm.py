from abc import abstractmethod
import numpy as np
from model.controllers.search_algorithm import SearchAlgorithm
from model.geometry.segment import Segment
from model.geometry.point import Point
from model.controllers.graph import Node


class SamplingBased(SearchAlgorithm):

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 margin=0.2,
                 iterations_per_step=1,
                 max_iterations=5000,
                 dynamic=False,
                 goal_sample_rate=0.05,
                 ):

        self.nodes = []
        self.edges = []

        self.goal_sample_rate = goal_sample_rate

        super().__init__(
            world_map,
            start,
            margin=margin,
            iterations_per_step=iterations_per_step,
            max_iterations=max_iterations,
            dynamic=dynamic,
        )

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.point.x - node_start.point.x
        dy = node_end.point.y - node_start.point.y
        return node_start.point.distance(node_end.point), np.arctan2(dy, dx)

    @staticmethod
    def compute_cost(node):
        n = node
        cost = 0.0
        while n.parent:
            cost += n.point.distance(n.parent.point)
            n = n.parent
        return cost

    def update_draw_list(self):
        # Overload the method to empty the draw_list first, getting rid of old segments.
        self.draw_list = []
        for node in self.nodes:
            self.draw_list.append(node.point)
            if node.parent is not None and node.parent.point is not None:
                self.draw_list.append(Segment(node.parent.point, node.point))

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            x = np.random.uniform(self.world_map.map_boundaries[0], self.world_map.map_boundaries[2])
            y = np.random.uniform(self.world_map.map_boundaries[1], self.world_map.map_boundaries[3])
        else:
            x, y = self.world_map.goal

        return Node(Point(x, y))

    def distance_to_goal(self, node):
        return node.point.distance(self.world_map.goal)

    def reset(self):
        self.nodes = []
        self.edges = []
        super().reset()

    @abstractmethod
    def step_search(self):
        pass

