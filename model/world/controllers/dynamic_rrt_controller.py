import math
import numpy as np

from model.geometry.point import Point
from model.geometry.segment import Segment
from model.world.controllers.controller import Controller


class Node:
    def __init__(self, coordinates):
        self.point = Point(coordinates[0], coordinates[1])
        self.parent = None
        self.valid = True

    @property
    def x(self):
        return self.point.x

    @property
    def y(self):
        return self.point.y

    def distance(self, other_node):
        return self.point.distance(other_node.point)


class DynamicRRTController(Controller):

    def __init__(self, robot, map, goal_sample_rate=0.05, max_iterations=8000, iterations=1):
        super().__init__(robot, map, iterations)

        # Percentage with which we use the goal as new point
        self.goal_sample_rate = goal_sample_rate

        self.max_iterations = max_iterations
        self.current_iteration = 0

        self.start = Node(robot.current_pose.as_point())
        self.goal = Node(map.goal)

        self.nodes = [self.start]

        self.draw_list = []

    def _search(self):

        if not self.has_path() and not self.is_robot_at_goal():

            if self.current_iteration < self.max_iterations:

                self.current_iteration += 1

                node_rand = self.generate_random_node()
                node_near = self.nearest_node(self.nodes, node_rand)
                node_new = self.new_state(node_near, node_rand)

                if node_new and self.is_collision_free(node_near, node_new):
                    self.nodes.append(node_new)
                    dist, _ = self.get_distance_and_angle(node_new, self.goal)

                    if dist <= self.map.discretization_step:
                        self.new_state(node_new, self.goal)
                        return self.extract_path(node_new)

                # Add the branches to the draw_list
                self.draw_list.append(Segment(node_new.point, node_new.parent.point))

    def extend(self, node):
        nearest_node = self.nearest_node(self.nodes, node)
        angle = np.arctan2(node.point.y - nearest_node.y, node.point.x - nearest_node.x)
        new_x = nearest_node.x + self.map.discretization_step * np.cos(angle)
        new_y = nearest_node.y + self.map.discretization_step * np.sin(angle)
        new_node = Node(Point(new_x, new_y))

        if not self.is_collision_free(nearest_node, new_node):
            return None

        new_node.parent = nearest_node
        self.nodes.append(new_node)
        return new_node

    def is_collision_free(self, node1, node2):
        return True

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return node_start.distance(node_end), math.atan2(dy, dx)

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select the a random point in space
            x = np.random.uniform(-self.map.obs_max_dist, self.map.obs_max_dist)
            y = np.random.uniform(-self.map.obs_max_dist, self.map.obs_max_dist)
            x = round(x / self.map.discretization_step) * self.map.discretization_step
            y = round(y / self.map.discretization_step) * self.map.discretization_step
        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return Node((x, y))

    def nearest_node(self, node_list, n):
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

    def reset(self):
        self.current_iteration = 0

        self.start = Node(self.robot.current_pose.as_point)
        self.goal = Node(self.map.goal)

        self.nodes = [self.start]

        self.draw_list = []
        self.path = []

    def get_draw_list(self):
        return self.draw_list
