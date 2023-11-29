import math
import numpy as np

from model.geometry.point import Point
from model.geometry.segment import Segment
from model.geometry.polygon import Polygon
from model.geometry.intersection import *
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

    def __str__(self):
        return f'Node({self.point})'

    def __repr__(self):
        return self.__str__()


class Edge(Segment):

    def __init__(self, node_parent, node_child):
        super().__init__(node_parent.point, node_child.point)
        self.node_parent = node_parent
        self.node_child = node_child


class DynamicRRTController(Controller):

    def __init__(self,
                 robot,
                 map,
                 step_len=0.2,
                 goal_sample_rate=0.05,
                 waypoint_sample_rate=0.05,
                 max_iterations=8000,
                 iterations=1,
                 discretization_step=0.2
                 ):

        #  We use the discretization step to build a buffer for each segment and check intersections
        super().__init__(robot, map, iterations, discretization_step)

        # Percentage with which we use the goal as new point
        self.goal_sample_rate = goal_sample_rate

        # Percentage with which we select a waypoint during replanning
        self.waypoint_sample_rate = waypoint_sample_rate

        # Maximum number of iterations (time constraint)
        self.max_iterations = max_iterations

        # The obstacles are disabled until the first path is found
        self.moving_obstacles_enabled = False

        self.step_len = step_len

        # Start and goal nodes
        self.start = None
        self.goal = None

        self.nodes = []
        self.edges = []
        self.waypoints = []

        self._init()

    def _init(self):

        self.start = Node(self.robot.current_pose.as_point())
        self.goal = Node(self.map.goal)

        self.nodes = [self.start]
        self.waypoints = []
        self.edges = []

        self.current_iteration = 0

        # Disable moving obstacles until the path isn't found
        self.map.disable_moving_obstacles()
        self.moving_obstacles_enabled = False

    def _search(self):

        """
        This will do for now, I will refactor everything in the controller module

        if not has_path():
            if has_waypoints():
                step_replan()
            else:
                step_plan()
        else:
            # Has path
            if is_path_valid():
                # Remove invalid branches that are not in the path
                trim()
            else:
                # Path is invalid, stop robot and recompute it
                trim()
                path = []
        """


        if not self.has_path() and not self.is_robot_at_goal():

            if self.current_iteration < self.max_iterations:

                self.current_iteration += 1
                self._step_plan()

        elif self.has_path():

            if not self.moving_obstacles_enabled:
                self.map.enable_moving_obstacles()
                self.moving_obstacles_enabled = True

            self.invalidate()
            if self.is_path_invalid():
                self.trim()
                print(f'{self.waypoints}')
                # self._step_replan()
            else:
                self.trim()

    def _step_plan(self):

        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            self.nodes.append(node_new)
            self.edges.append(Edge(node_near, node_new))
            dist = self.distance_to_goal(node_new)

            if dist <= self.discretization_step:
                # self.new_state(node_new, self.goal)
                self.extract_path(node_new)
                self.extract_waypoints(node_new)
                return

            # Add the branches to the draw_list
            self.draw_list.append(Segment(node_new.point, node_new.parent.point))
            self.draw_list.append(node_new.point)

    def _step_replan(self):

        node_rand = self.generate_random_node_replanning()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            self.nodes.append(node_new)
            self.edges.append(Edge(node_near, node_new))
            dist = self.distance_to_goal(node_new)

            if dist <= self.discretization_step:
                self.new_state(node_new, self.goal)
                self.extract_path(node_new)
                self.extract_waypoints(node_new)
                return

            # Add the branches to the draw_list
            self.draw_list.append(Segment(node_new.point, node_new.parent.point))
            self.draw_list.append(node_new.point)

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select a random point in space
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return Node((x, y))

    def generate_random_node_replanning(self):
        p = np.random.random()
        if p < self.goal_sample_rate:
            return self.goal
        elif self.goal_sample_rate < p < self.goal_sample_rate + self.waypoint_sample_rate:
            return self.waypoints[np.random.randint(0, len(self.waypoints) - 1)]
        else:
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            return Node(Point(x, y))

    def extract_waypoints(self, node):
        self.waypoints = [self.goal]
        node_now = node
        while node_now.parent is not None:
            node_now = node_now.parent
            self.waypoints.append(node_now)

    def trim(self):
        """
        If a node is invalid, cascading invalidate all its children
        """
        for i in range(1, len(self.nodes)):
            node = self.nodes[i]
            if not node.parent.valid:
                node.valid = False

        self.nodes = [node for node in self.nodes if node.valid]
        self.edges = [Edge(node.parent, node) for node in self.nodes[1: len(self.nodes)]]

        # Update draw_list
        self.draw_list = [Segment(node.parent.point, node.point) for node in self.nodes[1: len(self.nodes)]]

    def invalidate(self):
        """
        If an edge in the path is obstructed by an obstacle, invalidate is child
        """
        invalidate_result = False
        for edge in self.edges:
            if self.check_collision(edge.node_parent.point, edge.node_child.point):
                edge.node_child.valid = False
                invalidate_result = True

        return invalidate_result

    def is_path_invalid(self):
        for node in self.waypoints:
            if not node.valid:
                return True
        return False

    def nearest_neighbor(self, n):
        return min(self.nodes, key=lambda nd: nd.distance(n))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.discretization_step, dist)
        new_x = node_start.x + dist * math.cos(theta)
        new_y = node_start.y + dist * math.sin(theta)
        new_y = round(new_y / self.discretization_step, 2) * self.discretization_step
        new_x = round(new_x / self.discretization_step, 2) * self.discretization_step
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

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return node_start.distance(node_end), math.atan2(dy, dx)
