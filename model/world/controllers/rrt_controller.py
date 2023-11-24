import math
import numpy as np

from model.geometry.point import Point
from model.geometry.segment import Segment
from model.world.controllers.controller import Controller

np.random.seed()


class Node:
    def __init__(self, coordinates):
        self.point = Point(coordinates[0], coordinates[1])
        self.parent = None

    @property
    def x(self):
        return self.point.x

    @property
    def y(self):
        return self.point.y

    def distance(self, other_node):
        return self.point.distance(other_node.point)


class RRTController(Controller):

    def __init__(self, robot, map, goal_sample_rate, max_iterations=8000, iterations=1):
        super().__init__(robot, map, iterations)
        self.goal_sample_rate = goal_sample_rate
        self.vertex = [Node(robot.current_pose.as_point())]
        self.draw_list = []
        self.max_iterations = max_iterations
        self.current_iteration = 0

    def _search(self):

        if not self.has_path():

            if self.current_iteration < self.max_iterations:

                self.current_iteration += 1

                node_rand = self.generate_random_node(self.goal_sample_rate)
                node_near = self.nearest_neighbor(self.vertex, node_rand)
                node_new = self.new_state(node_near, node_rand)

                if node_new:  # and not self.is_path_obstructed(node_near, node_new):
                    self.vertex.append(node_new)
                    dist = self.distance_to_goal(node_new)

                    if dist <= self.map.discretization_step:  # and not self.is_path_obstructed(node_new, self.map.goal):
                        return self.extract_path(node_new)

                    # Add the branches to the draw_list
                    self.draw_list.append(Segment(node_new.point, node_new.parent.point))

    def generate_random_node(self, goal_sample_rate):

        if np.random.random() > goal_sample_rate:
            x = np.random.uniform(-self.map.obs_max_dist, self.map.obs_max_dist)
            y = np.random.uniform(-self.map.obs_max_dist, self.map.obs_max_dist)
            x = round(x / self.map.discretization_step) * self.map.discretization_step
            y = round(y / self.map.discretization_step) * self.map.discretization_step
        else:
            x, y = self.map.goal

        return Node((x, y))

    def nearest_neighbor(self, node_list, n):
        return min(node_list, key=lambda nd: nd.distance(n))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.map.discretization_step, dist)
        new_x = node_start.x + dist * math.cos(theta)
        new_y = node_start.y + dist * math.sin(theta)
        new_y = round(new_y / self.map.discretization_step, 2) * self.map.discretization_step
        new_x = round(new_x / self.map.discretization_step, 2) * self.map.discretization_step
        node_new = Node((new_x, new_y))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        self.path = [Point(self.map.goal[0], self.map.goal[1])]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.x, node_now.y))

        self.path = self.path[::-1]

    def distance_to_goal(self, node):
        return math.hypot(node.x - self.map.goal[0], node.y - self.map.goal[1])

    """
    def is_path_obstructed(self, node_start, node_end):
        line_segment = Segment((node_start.x, node_start.y), (node_end.x, node_end.y))
        return self.map.check_collision(line_segment.start, line_segment.end)
    """

    def reset(self):
        self.path = []
        self.vertex = [Node(self.robot.current_pose.as_point())]
        self.draw_list = []

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return node_start.distance(node_end), math.atan2(dy, dx)

    def get_draw_list(self):
        return self.draw_list
