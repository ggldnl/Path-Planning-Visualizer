from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node

from model.geometry.segment import Segment
from model.geometry.point import Point

import numpy as np


class IRRTStar(SamplingBased):

    def __init__(self,
                 map,
                 start=Point(0, 0),
                 boundary=0.2,
                 iterations=1,
                 step_length=0.2,
                 search_radius=0.5,
                 max_iterations=1000,
                 goal_sample_rate=0.05,
                 ):

        self.step_length = step_length
        self.search_radius = search_radius
        self.goal_sample_rate = goal_sample_rate

        self.max_iterations = max_iterations
        self.current_iteration = 0

        super().__init__(map, start, boundary, iterations)

    def heuristic(self, point):
        return 0

    def pre_search(self):

        self.nodes = [Node(self.start)]
        self.edges = []
        self.current_iteration = 0

        self.path = []
        self.draw_list = []

        self.map.disable_moving_obstacles()

    def step_search(self):
        theta, dist, x_center, C, x_best = self.init()
        c_best = np.inf

        self.current_iteration += 1

        if self.X_soln:
            cost = {node: self.Cost(node) for node in self.X_soln}
            x_best = min(cost, key=cost.get)
            c_best = cost[x_best]


        x_rand = self.Sample(c_best, dist, x_center, C)
        x_nearest = self.Nearest(self.V, x_rand)
        x_new = self.Steer(x_nearest, x_rand)

        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):

            nodes_near = self.near_neighbor(node_new)
            c_min = self.Cost(x_nearest) + self.Line(x_nearest, x_new)

            self.nodes.append(node_new)

            self.choose_parent(nodes_near)
            self.rewire(nodes_near)

            self.update_draw_list(None)

    def post_search(self):

        index = self.search_goal_parent()
        if index > 0:
            self.extract_path(self.nodes[index])

    def can_run(self):
        return self.current_iteration < self.max_iterations

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

        # return len(self.vertex) - 1
        return -1

    def find_neighborhood(self, node_new):

        # n = len(self.nodes) + 1
        # r = min(self.search_radius * np.sqrt((np.log(n) / n)), self.discretization_step)
        r = self.search_radius

        dist_table = [np.hypot(nd.point.x - node_new.point.x, nd.point.y - node_new.point.y) for nd in self.nodes]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not node_new.point == self.map.goal and
                            not self.check_collision(node_new.point, self.nodes[ind].point)]

        return dist_table_index

    def has_path(self):
        return len(self.path) > 0 and self.path[-1] == self.map.goal

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
    def compute_cost(node):
        n = node
        cost = 0.0
        while n.parent:
            cost += n.point.distance(n.parent.point)
            n = n.parent
        return cost

    def update_draw_list(self, placeholder):
        # Overload the method to empty the draw_list first, getting rid of old segments.
        # Placeholder is discarded
        self.draw_list = []
        for node in self.nodes:
            if node.parent is not None:
                self.draw_list.append(node.point)
                self.draw_list.append(Segment(node.parent.point, node.point))
