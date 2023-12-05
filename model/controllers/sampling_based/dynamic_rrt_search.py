from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node
from model.controllers.graph import Edge

from model.geometry.segment import Segment
from model.geometry.point import Point
from model.geometry.polygon import Polygon

import numpy as np


class ValidNode(Node):
    def __init__(self, point, parent=None, cost=0, heuristic=0, valid=True):
        super().__init__(point, parent, cost, heuristic)
        self.valid = valid


class DynamicRRT(SamplingBased):

    def __init__(self,
                 map,
                 start=Point(0, 0),
                 boundary=0.2,
                 step_length=0.2,
                 max_iterations=5000,
                 goal_sample_rate=0.05,
                 waypoint_sampling_rate=0.5
                 ):

        self.step_length = step_length
        self.max_iterations = max_iterations
        self.goal_sample_rate = goal_sample_rate
        self.waypoint_sample_rate = waypoint_sampling_rate

        self.current_iteration = 0

        self.planning_done = False

        # The obstacles are disabled until the first path is found
        self.moving_obstacles_enabled = False
        self.moving_obstacles_status = False
        self.moving_obstacles_status_before = self.moving_obstacles_status

        self.waypoints = []  # Cached nodes
        self.path_nodes = set()  # Nodes forming the path
        self.last_valid = None  # Points to the last valid point in the path

        super().__init__(map, start, boundary)

    def heuristic(self, point):
        return point.distance(self.map.goal)

    def pre_search(self):

        self.nodes = [ValidNode(self.start)]
        self.edges = []
        self.waypoints = []
        self.path_nodes = set()
        self.last_valid = None  # Points to the last valid point in the path

        self.current_iteration = 0

        self.planning_done = False

        self.path = []
        self.draw_list = []

        self.map.disable_moving_obstacles()
        self.moving_obstacles_enabled = False
        self.moving_obstacles_status = False
        self.moving_obstacles_status_before = self.moving_obstacles_status

    def can_run(self):
        """
        Running conditions:
        1. iterations left -> self.current_iteration < self.max_iterations
        2. not found initial plan -> not self.planning_done
        3. reached the goal -> self.planning_done and len(self.path) == 1 and self.path[0] == self.map.goal
        """
        return self.current_iteration < self.max_iterations

    def handle_moving_obstacles(self):
        """
        Moving obstacles should be disabled when the search algorithm is searching
        for a path and enabled otherwise.
        """
        if self.moving_obstacles_status != self.moving_obstacles_status_before:
            if self.moving_obstacles_enabled:
                print('Moving obstacles disabled')
                self.map.disable_moving_obstacles()
            else:
                print('Moving obstacles enabled')
                self.map.enable_moving_obstacles()
            self.moving_obstacles_enabled = not self.moving_obstacles_enabled
            self.moving_obstacles_status_before = self.moving_obstacles_status

    def step_search(self):

        self.current_iteration += 1

        # Enable/disable moving obstacles
        self.handle_moving_obstacles()

        # print('Stepping search...')
        # print(f'Has path    : {self.has_path()}')
        # print(f'Path        : {self.path}')
        # print(f'Waypoints   : {self.waypoints}')
        # print(f'Iteration   : {self.current_iteration}')
        # print(f'Path invalid: {self.is_path_invalid()}')
        # print()

        if not self.planning_done:
            self.step_planning()
        else:
            self.replanning()

        self.handle_moving_obstacles()

        # print('-' * 50)

    def replanning(self):

        # Check if some edges are now invalid
        self.invalidate_nodes()

        if self.is_path_invalid():

            # Disable moving obstacles in case they were disabled
            self.moving_obstacles_status = False

            # The path is invalid, we need to progressively find another path by calling replan at each step,
            # but we also need to call trim once before the replan and to invalidate only part of the initial
            # path. Trim will trim the invalid edges and set last_valid that is the last valid node of the
            # path. Replan will then reconstruct the path starting from the goal and ending to last_valid.

            # If last_valid does not exist, then the trim hasn't happened yet
            if not self.last_valid:
                self.trim()
            else:
                self.step_replanning()

    def is_path_invalid(self):
        for node in self.waypoints:
            if not node.valid:
                return True
        return False

    def trim(self):
        """
        If a node is invalid, cascading invalidate all its children
        """
        for i in range(1, len(self.nodes)):
            node = self.nodes[i]
            if not node.parent.valid:
                node.valid = False

        # Find last_valid
        self.find_last_valid()

        self.nodes = [node for node in self.nodes if node.valid]
        self.edges = [Edge(node.parent, node) for node in self.nodes[1: len(self.nodes)]]
        self.waypoints = [node for node in self.waypoints if not node.valid]

        # Update draw_list
        self.update_draw_list(None)

    def find_last_valid(self):
        """
        Find the last valid node in the path and set the last_valid variable
        """
        print(f'Path before: {self.path}')
        last_valid_index = -1
        for i, point in enumerate(self.path):
            current_node = None
            for node in self.path_nodes:
                if node.point == point:
                    current_node = node
                    break
            if not current_node.valid:
                self.last_valid = current_node
                last_valid_index = i
                break

        self.path = self.path[:last_valid_index]
        print(f'Path now   : {self.path}')
        print('-' * 50)
        print()

    def invalidate_nodes(self):
        """
        If an edge is obstructed by an obstacle, invalidate its child
        """
        for edge in self.edges:
            if self.check_collision(edge.parent_node.point, edge.child_node.point):
                edge.child_node.valid = False

    def step_planning(self):

        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            self.nodes.append(node_new)
            self.edges.append(Edge(node_near, node_new))
            dist = self.distance_to_goal(node_new)

            if dist <= self.step_length:
                self.extract_path(node_new)
                self.extract_waypoints(node_new)
                return

            # Update drawing list
            self.update_draw_list(None)

    def step_replanning(self):

        node_rand = self.generate_random_node_replanning()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            self.nodes.append(node_new)
            self.edges.append(Edge(node_near, node_new))
            dist = self.distance_to_goal(node_new)

            if dist <= self.step_length:
                self.extract_path_replanning(node_new)
                self.extract_waypoints(node_new)
                return

            # Update drawing list
            self.update_draw_list(None)

    def check_collision(self, start, end):
        if start == end:
            return False
        if end.distance(self.map.goal) < self.map.min_goal_clearance:
            return False
        return super().check_collision(start, end)

    def extract_waypoints(self, node):
        self.waypoints = [ValidNode(self.map.goal)]
        node_now = node
        while node_now.parent is not None:
            node_now = node_now.parent
            self.waypoints.append(node_now)

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select a random point in space
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return ValidNode(Point(x, y))

    def generate_random_node_replanning(self):
        p = np.random.random()
        if p < self.goal_sample_rate:
            return ValidNode(self.map.goal)
        elif self.goal_sample_rate < p < self.goal_sample_rate + self.waypoint_sample_rate:
            return self.waypoints[np.random.randint(0, len(self.waypoints) - 1)]
        else:
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            return ValidNode(Point(x, y))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_length, dist)
        new_x = node_start.point.x + dist * np.cos(theta)
        new_y = node_start.point.y + dist * np.sin(theta)

        # new_x = round(new_x / self.step_length, 2) * self.step_length
        # new_y = round(new_y / self.step_length, 2) * self.step_length

        node_new = ValidNode(Point(new_x, new_y))
        node_new.parent = node_start

        return node_new

    def nearest_neighbor(self, n):
        return min(self.nodes, key=lambda nd: nd.point.distance(n.point))

    def distance_to_goal(self, node):
        return node.point.distance(self.map.goal)

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.point.x - node_start.point.x
        dy = node_end.point.y - node_start.point.y
        return node_start.point.distance(node_end.point), np.arctan2(dy, dx)

    def extract_path(self, node_end):
        self.path = [self.map.goal]
        self.path_nodes = {ValidNode(self.map.goal)}
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(node_now.point)
            self.path_nodes.add(node_now)

        self.path = self.path[::-1]

        # Set planning as done
        self.planning_done = True

        # Enable moving obstacles
        self.moving_obstacles_status = True

    def extract_path_replanning(self, node_end):
        self.path = [self.map.goal]
        self.path_nodes = {ValidNode(self.map.goal)}
        node_now = node_end

        while node_now.parent is not None and node_now != self.last_valid:
            node_now = node_now.parent
            self.path.append(node_now.point)
            self.path_nodes.add(node_now)

        self.path = self.path[::-1]

        # Enable moving obstacles
        self.moving_obstacles_status = True

        # Clear last_valid to let trim happen again if the path is invalid
        self.last_valid = None

    def update_draw_list(self, placeholder):
        self.draw_list = []
        for node in self.nodes:
            if node.parent is not None:
                self.draw_list.append(node.point)
                self.draw_list.append(Segment(node.parent.point, node.point))

                """
                # Add segment buffers to the drawing list 
                line = Segment(node.parent.point, node.point)
                buffer = Polygon.get_segment_buffer(line, left_margin=self.boundary/2, right_margin=self.boundary/2)
                self.draw_list.append(buffer)
                """
