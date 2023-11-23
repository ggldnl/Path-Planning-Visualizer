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
    def __init__(self, robot, map, step_size, goal_sample_rate, iter_max):
        super().__init__(robot, map)
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [Node(robot.current_pose.as_point())]

    def search(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new:  # and not self.is_path_obstructed(node_near, node_new):
                self.vertex.append(node_new)
                dist = self.distance_to_goal(node_new)

                if dist <= self.step_size:  # and not self.is_path_obstructed(node_new, self.map.goal):
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.EPS

        if np.random.random() > goal_sample_rate:
            x = np.random.uniform(0 + delta, 20 - delta)
            y = np.random.uniform(0 + delta, 20 - delta)
            x = x // self.step_size * self.step_size
            y = y // self.step_size * self.step_size
        else:
            x, y = self.map.goal

        return Node((x, y))

    def nearest_neighbor(self, node_list, n):
        return min(node_list, key=lambda nd: nd.distance(n))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_size, dist)
        new_x = node_start.x + dist * math.cos(theta)
        new_y = node_start.y + dist * math.sin(theta)
        new_y = new_y // self.step_size * self.step_size
        new_x = new_x // self.step_size * self.step_size
        node_new = Node((new_x, new_y))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        self.path = [Point(self.map.goal[0], self.map.goal[1])]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.x, node_now.y))

        return

    def distance_to_goal(self, node):
        return math.hypot(node.x - self.map.goal[0], node.y - self.map.goal[1])

    def is_path_obstructed(self, node_start, node_end):
        line_segment = Segment((node_start.x, node_start.y), (node_end.x, node_end.y))
        return self.map.check_collision(line_segment.start, line_segment.end)

    def reset(self):
        self.path = []
        self.vertex = [Node(self.robot.current_pose.as_point())]

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return node_start.distance(node_end), math.atan2(dy, dx)
