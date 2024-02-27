import random

from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node
from model.geometry.ellipse import Ellipse

from model.geometry.segment import Segment
from model.geometry.point import Point
import math
import numpy as np


class InformedRRTStar(SamplingBased):

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 margin=0.2,
                 iterations_per_step=1,
                 step_length=0.2,
                 search_radius=0.5,
                 max_iterations=1000,
                 goal_sample_rate=0.05,
                 ):
        super().__init__(world_map, start, margin, iterations_per_step, max_iterations)

        self.step_length = step_length
        self.search_radius = search_radius
        self.goal_sample_rate = goal_sample_rate
        self.ellipse = None
        self.need_for_path = True

    def heuristic(self, point):
        return 0

    def pre_search(self):

        self.nodes = [Node(self.start)]
        self.edges = []
        self.need_for_path = True
        self.ellipse = None

    def step_search(self):

        if self.need_for_path:
            self.step_planning()
        else:
            self.step_replanning()

    def step_planning(self):
        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            neighbor_index = self.find_neighborhood(node_new)

            self.nodes.append(node_new)

            if neighbor_index:
                self.choose_parent(node_new, neighbor_index)
                self.rewire(node_new, neighbor_index)

            dist = self.distance_to_goal(node_new)

            if dist <= self.step_length:
                self.extract_path(node_new)

            self.update_draw_list()

    def step_replanning(self):
        node_rand = self.generate_random_node_replanning()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            neighbor_index = self.find_neighborhood(node_new)

            self.nodes.append(node_new)

            if neighbor_index:
                self.choose_parent(node_new, neighbor_index)
                self.rewire(node_new, neighbor_index)

            dist = self.distance_to_goal(node_new)

            if dist <= self.step_length:
                self.extract_path(node_new)

            self.update_draw_list()

    def generate_random_node_replanning(self):
        if self.path_nodes:
            if self.ellipse is None:
                focus1 = self.path_nodes[0]
                focus2 = self.world_map.goal
                self.ellipse = Ellipse.from_path_points(focus1, focus2, focus1.distance(focus2) + 2)

            random_node_inside_ellipse = Node(Point.from_dict(self.ellipse.generate_point_inside()))
            return random_node_inside_ellipse

    def post_search(self):
        if not self.need_for_path:
            self.path = [point for point in self.path_nodes]

    def check_collision(self, point_start, point_end):
        if point_start == point_end:
            return False
        return super().check_collision(point_start, point_end)

    def search_goal_parent(self):
        dist_list = [self.distance_to_goal(n) for n in self.nodes]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.search_radius]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.compute_cost(self.nodes[i]) for i in node_index
                         if not self.nodes[i].point == self.world_map.goal and
                         not self.check_collision(self.nodes[i].point, self.world_map.goal)]
            return node_index[int(np.argmin(cost_list))]
        return -1

    def find_neighborhood(self, node_new):
        dist_table = [np.hypot(nd.point.x - node_new.point.x, nd.point.y - node_new.point.y) for nd in self.nodes]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= self.search_radius and
                            not node_new.point == self.world_map.goal and
                            not self.check_collision(node_new.point, self.nodes[ind].point)]

        return dist_table_index

    def has_path(self):
        return len(self.path) > 0 and self.path[-1] == self.world_map.goal

    def nearest_neighbor(self, n):
        return min(self.nodes, key=lambda nd: nd.point.distance(n.point))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_length, dist)
        new_x = node_start.point.x + dist * np.cos(theta)
        new_y = node_start.point.y + dist * np.sin(theta)

        node_new = Node(Point(new_x, new_y))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        self.need_for_path = False
        self.path_nodes = [self.world_map.goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path_nodes.append(Point(node_now.point.x, node_now.point.y))

        self.path_nodes = self.path_nodes[::-1]

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.nodes[i], node_new) for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.nodes[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.nodes[i]

            if self.compute_cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        return self.compute_cost(node_start) + dist

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

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            x = np.random.uniform(-2 * self.world_map.obs_max_dist, 0) + self.world_map.obs_max_dist
            y = np.random.uniform(-2 * self.world_map.obs_max_dist, 0) + self.world_map.obs_max_dist
        else:
            x, y = self.world_map.goal

        return Node(Point(x, y))

    def distance_to_goal(self, node):
        return node.point.distance(self.world_map.goal)
