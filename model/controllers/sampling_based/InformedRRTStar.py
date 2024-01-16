import random

from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node

from model.geometry.segment import Segment
from model.geometry.point import Point
import math
import numpy as np


class InformedRRTStar(SamplingBased):

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 boundary=0.2,
                 iterations=1,
                 step_length=0.2,
                 search_radius=0.5,
                 max_iterations=1000,
                 goal_sample_rate=0.05,
                 ):
        super().__init__(world_map, start, boundary, iterations, max_iterations)

        self.step_length = step_length
        self.search_radius = search_radius
        self.goal_sample_rate = goal_sample_rate

    def heuristic(self, point):
        return 0

    def pre_search(self):

        self.nodes = [Node(self.start)]
        self.edges = []
        self.current_iteration = 0

        self.path = []
        self.draw_list = []

        self.map.disable_moving_obstacles()
        self.planning_done = False

    def step_search(self):
        self.current_iteration += 1

        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            nodes_near_index = self.find_neighborhood(node_new)
            self.choose_parent(node_new, nodes_near_index)
            self.rewire(node_new, nodes_near_index)

            if self.distance_to_goal(node_new) < self.step_length:
                if not self.check_collision(node_new, self.map.goal):
                    self.nodes.append(node_new)

            self.update_draw_list(None)

    def post_search(self):

        index = self.search_goal_parent()
        if index > 0:
            self.extract_path(self.nodes[index])

    def check_collision(self, point_start, point_end):
        if point_start == point_end:
            return False
        return super().check_collision(point_start, point_end)

    def search_goal_parent(self):
        dist_list = [self.distance_to_goal(n) for n in self.nodes]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.search_radius]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.compute_cost(self.nodes[i]) for i in node_index
                         if not self.nodes[i].point == self.map.goal and
                         not self.check_collision(self.nodes[i].point, self.map.goal)]
            return node_index[int(np.argmin(cost_list))]
        return -1

    def find_neighborhood(self, node_new):
        dist_table = [np.hypot(nd.point.x - node_new.point.x, nd.point.y - node_new.point.y) for nd in self.nodes]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= self.search_radius and
                            not node_new.point == self.map.goal and
                            not self.check_collision(node_new.point, self.nodes[ind].point)]

        return dist_table_index

    def has_path(self):
        return len(self.path) > 0 and self.path[-1] == self.map.goal



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
        self.path = [Point(self.map.goal[0], self.map.goal[1])]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.point.x, node_now.point.y))

        self.path = self.path[::-1]

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.nodes[i], node_new) for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.nodes[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.nodes[i]

            if self.compute_cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new


