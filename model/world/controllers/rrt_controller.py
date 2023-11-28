import numpy as np

from model.geometry.point import Point
from model.geometry.segment import Segment
from model.geometry.polygon import Polygon
from model.world.controllers.controller import Controller


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

    def __str__(self):
        return f'Node({self.point})'

    def __repr__(self):
        return self.__str__()


class RRTController(Controller):

    def __init__(self,
                 robot,
                 map,
                 goal_sample_rate=0.05,
                 max_iterations=8000,
                 iterations=1,
                 discretization_step=0.2
                 ):

        #  We use the discretization step to build a buffer for each segment and check intersections
        super().__init__(robot, map, iterations, discretization_step)

        # Percentage with which we use the goal as new point
        self.goal_sample_rate = goal_sample_rate

        # Maximum number of iterations (time constraint)
        self.max_iterations = max_iterations

        self._init()

    def _init(self):

        self.vertex = [Node(self.robot.current_pose.as_point())]
        self.edges = []

        self.current_iteration = 0

        # Disable moving obstacles until the path isn't found
        self.map.disable_moving_obstacles()

    def _search(self):

        if not self.has_path() and not self.is_robot_at_goal():

            if self.current_iteration < self.max_iterations:

                self.current_iteration += 1

                node_rand = self.generate_random_node()
                node_near = self.nearest_neighbor(node_rand)
                node_new = self.new_state(node_near, node_rand)

                if node_new and not self.check_collision(node_near.point, node_new.point):
                    self.vertex.append(node_new)
                    dist = self.distance_to_goal(node_new)

                    if dist <= self.map.discretization_step:
                        return self.extract_path(node_new)

                    # Add the branches to the draw_list
                    self.draw_list.append(node_new.point)
                    self.draw_list.append(Segment(node_new.point, node_new.parent.point))

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select a random point in space
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return Node((x, y))

    def nearest_neighbor(self, n):
        return min(self.vertex, key=lambda nd: nd.distance(n))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.map.discretization_step, dist)
        new_x = node_start.x + dist * np.cos(theta)
        new_y = node_start.y + dist * np.sin(theta)
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
        return node.point.distance(self.map.goal)

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return node_start.distance(node_end), np.arctan2(dy, dx)
