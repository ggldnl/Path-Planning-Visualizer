from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node

from model.geometry.point import Point

import numpy as np


class RRT(SamplingBased):

    def __init__(self,
                 map,
                 start=Point(0, 0),
                 boundary=0.2,
                 max_iterations=8000,
                 step_length=0.5,
                 goal_sample_rate=0.05,
                 ):

        self.max_iterations = max_iterations
        self.step_length = step_length
        self.goal_sample_rate = goal_sample_rate

        self.current_iteration = 0

        super().__init__(map, start, boundary)

    def heuristic(self, point):
        return 0

    def init(self):

        self.nodes = [Node(self.start)]
        self.edges = []
        self.current_iteration = 0

        self.path = []
        self.draw_list = []

        self.map.disable_moving_obstacles()

    def step(self):

        if self.current_iteration < self.max_iterations:

            self.current_iteration += 1

            node_rand = self.generate_random_node()
            node_near = self.nearest_neighbor(node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.check_collision(node_near.point, node_new.point):
                self.nodes.append(node_new)
                dist = self.distance_to_goal(node_new)

                if dist <= self.step_length:
                    return self.extract_path(node_new)

                # Update drawing list
                self.update_draw_list(node_new)

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select a random point in space
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist

        # Goal bias
        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return Node(Point(x, y))

    def nearest_neighbor(self, n):
        return min(self.nodes, key=lambda nd: nd.point.distance(n.point))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_length, dist)
        new_x = node_start.point.x + dist * np.cos(theta)
        new_y = node_start.point.y + dist * np.sin(theta)

        # new_y = round(new_y / self.search_step, 2) * self.search_step
        # new_x = round(new_x / self.search_step, 2) * self.search_step

        node_new = Node(Point(new_x, new_y))
        node_new.parent = node_start

        return node_new

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.point.x - node_start.point.x
        dy = node_end.point.y - node_start.point.y
        return node_start.point.distance(node_end.point), np.arctan2(dy, dx)

    def distance_to_goal(self, node):
        return node.point.distance(self.map.goal)
    
    def extract_path(self, node_end):
        self.path = [Point(self.map.goal[0], self.map.goal[1])]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.point.x, node_now.point.y))

        self.path = self.path[::-1]
