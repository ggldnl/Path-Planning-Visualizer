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
                 dynamic=False,
                 max_iterations=5000,
                 goal_sample_rate=0.05,
                 ):

        self.nodes = []
        self.edges = []

        self.max_iterations = max_iterations
        self.current_iteration = 0
        self.goal_sample_rate = goal_sample_rate

        #                world_map, start, margin, iterations
        super().__init__(world_map, start, margin, iterations_per_step, dynamic)

    @property
    def map(self):
        return self.world_map

    @abstractmethod
    def heuristic(self, point):
        pass

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
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
        else:
            x, y = self.map.goal

        return Node(Point(x, y))

    def distance_to_goal(self, node):
        return node.point.distance(self.map.goal)

    def can_run(self):
        """
        Running conditions:
        1. iterations left -> self.current_iteration < self.max_iterations
        2. not found initial plan -> not self.planning_done
        3. reached the goal -> self.planning_done and len(self.path) == 1 and self.path[0] == self.world_map.goal
        """
        return self.current_iteration < self.max_iterations
