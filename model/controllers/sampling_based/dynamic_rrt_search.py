from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node

from model.geometry.segment import Segment
from model.geometry.point import Point

import numpy as np


class ValidNode(Node):
    def __init__(self, point, parent=None, cost=0, heuristic=0, valid=True):
        super().__init__(point, parent, cost, heuristic)
        self.valid = valid

    def __str__(self):
        return f'Node ({self.point}, {"v" if self.valid else "n"})'


class PathWrapper:

    def __init__(self, path=None):
        if path is None:
            self.path = []
        else:
            self.path = path

    def set_path(self, path):
        # print(f'PathWrapper: path set -> {self.path}')
        self.path = path

    def __getitem__(self, item):
        # print(f'PathWrapper: returning element {item} -> [{self.path[item].point}]')
        return self.path[item].point

    def pop(self, item):
        # print(f'PathWrapper: popping element {item}')
        self.path.pop(item)

    def __len__(self):
        return len(self.path)

    def get_as_valid_string(self):
        return ''.join(['v' if node.valid else '.' for node in self.path])

    def __str__(self):
        return self.path.__str__()

    def __repr__(self):
        return self.path.__repr__()


class DynamicRRT(SamplingBased):
    """
    Dynamic RRT (RRT with basic replanning). This algorithm is a simple
    RRT extension; once it finds a path, it stores the path in a backup
    list (waypoints). If the path is disrupted by an obstacle, it needs
    to be recomputed. The DynamicRRT recomputes the path by taking
    nodes from the waypoints cache when recomputing the path with a
    given probability. It has two phases: planning (finding the first path)
    and replanning (update the path if it is invalid). The replanning
    generates a path very similar to the previous one and this is the
    reason why this algorithm is not so great in our use case (obstacles
    moves on the plane, they don't pop up in random positions).
    """

    def __init__(self,
                 map,
                 start=Point(0, 0),
                 boundary=0.2,
                 iterations=1,
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
        self.path_nodes = []  # Nodes forming the path
        self.last_valid = None  # Points to the last valid point in the path
        self.node_current = None  # Points to the current node (first element of the path)

        # Uniform with the interface (it expects the path to contain points)
        self.path_wrapper = PathWrapper()

        super().__init__(map, start, boundary, iterations)

    @property
    def path(self):
        return self.path_wrapper

    @path.setter
    def path(self, new_path):
        self.path_wrapper.set_path(new_path)

    def heuristic(self, point):
        return point.distance(self.map.goal)

    def pre_search(self):

        self.nodes = [ValidNode(self.start)]
        self.waypoints = []
        self.path_nodes = []
        self.last_valid = None  # Points to the last valid point in the path
        self.node_current = None

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
                # print('Moving obstacles disabled')
                self.map.disable_moving_obstacles()
            else:
                # print('Moving obstacles enabled')
                self.map.enable_moving_obstacles()
            self.moving_obstacles_enabled = not self.moving_obstacles_enabled
            self.moving_obstacles_status_before = self.moving_obstacles_status

    def step_search(self):

        self.current_iteration += 1

        # print('Stepping search...')
        # print(f'Iteration   : {self.current_iteration}')
        # print(f'Has path    : {self.has_path()}')
        # print(f'Path        : {self.path}')
        # print(f'Waypoints   : {self.waypoints}')
        # print(f'Valid path  : {self.path.get_as_valid_string()}')
        # print(f'Path invalid: {self.is_path_invalid()}')

        if not self.planning_done:
            self.step_planning()
        else:

            self.invalidate_path()

            self.node_current = self.path[0]

            if self.is_path_invalid():

                # Disable the obstacles
                self.moving_obstacles_status = False

                # The path is invalid, we need to progressively find another path by calling replan at each step,
                # but we also need to call trim once before the replan and to invalidate only part of the initial
                # path. Trim will trim the invalid edges and set last_valid that is the last valid node of the
                # path. Replan will then reconstruct the path starting from the goal and ending to last_valid.

                # If last_valid does not exist, then the trim hasn't happened yet
                if not self.last_valid:
                    # self.propagate()
                    self.trim()
                else:
                    self.step_replanning()

        # Enable/disable moving obstacles
        self.handle_moving_obstacles()

        # print('-' * 50)

    def invalidate_path(self):
        """
        For each node in the path and its parent (previous one), check if there is a collision in between
        """
        for i in range(1, len(self.path_nodes)):
            # TODO: fix this (index out of bounds)
            if self.check_collision(self.path_nodes[i-1].point, self.path_nodes[i].point):
                # print(f'Invalidating node: {self.path_nodes[i]}')
                self.path_nodes[i].valid = False

    def propagate(self):
        """
        If a node is invalid, cascading invalidate all its children
        """
        for i in range(1, len(self.path_nodes)):
            node = self.path_nodes[i]
            if not node.valid:
                for j in range(i, len(self.path_nodes)):
                    self.path_nodes[j].valid = False
                break

    def trim(self):
        """
        1. Propagate invalid status to all the nodes that don't have a parent.
        2. Find the first occurrence of an invalid node in the path and trim the rest.
        The remaining, trimmed, nodes are used to create the waypoints list.
        """

        for i in range(1, len(self.nodes)):
            node = self.nodes[i]
            if not node.parent.valid:
                node.valid = False

        idx = -1
        for i in range(len(self.path_nodes)):
            node = self.path_nodes[i]
            if not node.valid:
                idx = i
                break

        self.waypoints = self.path_nodes[idx:]
        self.path_nodes = self.path_nodes[:idx]
        self.last_valid = self.path_nodes[-1]
        self.path_wrapper.set_path(self.path_nodes)
        self.nodes = [node for node in self.nodes if node.valid]

        self.update_draw_list(None)

    def is_path_invalid(self):
        if not self.has_path():
            return True
        for node in self.path_nodes:
            if not node.valid:
                return True
        return False

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

    def check_collision(self, start, end):
        if start == end:
            return False
        if end.distance(self.map.goal) < self.map.min_goal_clearance:
            return False
        return super().check_collision(start, end)

    # ------------------------------- planning step ------------------------------ #

    def step_planning(self):
        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            self.nodes.append(node_new)
            dist = self.distance_to_goal(node_new)

            if dist <= self.step_length:
                self.extract_path_and_waypoints(node_new)
                return

            # Update drawing list
            self.update_draw_list(None)

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select a random point in space
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return ValidNode(Point(x, y))

    def nearest_neighbor(self, n):
        return min(self.nodes, key=lambda nd: nd.point.distance(n.point))

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

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.point.x - node_start.point.x
        dy = node_end.point.y - node_start.point.y
        return node_start.point.distance(node_end.point), np.arctan2(dy, dx)

    def distance_to_goal(self, node):
        return node.point.distance(self.map.goal)

    def extract_path_and_waypoints(self, node_end):

        goal_node = ValidNode(self.map.goal)
        self.path_nodes = [goal_node]
        # self.waypoints = [goal_node]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path_nodes.append(node_now)
            # self.waypoints.append(node_now)

        # Reverse the path to make it go from start to goal
        self.path_nodes = self.path_nodes[::-1]

        # Set the path
        self.path_wrapper.set_path(self.path_nodes)

        # Set planning as done
        self.planning_done = True

        # Enable moving obstacles
        self.moving_obstacles_status = True

    # ------------------------------ replanning step ----------------------------- #

    def step_replanning(self):

        node_rand = self.generate_random_node_replanning()
        node_near = self.nearest_neighbor(node_rand)
        node_new = self.new_state(node_near, node_rand)

        if node_new and not self.check_collision(node_near.point, node_new.point):
            self.nodes.append(node_new)
            dist = self.distance_to_goal(node_new)

            if dist <= self.step_length:
                self.extract_path_and_waypoints_replanning(node_new)
                return

            # Update drawing list
            self.update_draw_list(None)

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

    def extract_path_and_waypoints_replanning(self, node_end):
        """
        Rebuild the path going back until the parent node is the last_valid node
        """

        goal_node = ValidNode(self.map.goal)
        new_path_nodes = [goal_node]
        node_now = node_end

        is_current_in_same_path = False
        while node_now.parent is not None:

            if node_now.point == self.node_current:
                is_current_in_same_path = True
                break

            node_now = node_now.parent
            new_path_nodes.append(node_now)

        if not is_current_in_same_path:
            # TODO: important! The path planning algorithm does not have the task
            #  of making the robot go back to the origin following the same points
            #  it followed before but backwards. This is the controller's job, but
            #  our simple controller does not take this into account and makes the
            #  robot automatically go to the origin (parent == Null) if a new path
            #  does not include a node_current
            pass

        # Reverse the path to make it go from start to goal
        new_path_nodes = new_path_nodes[::-1]
        self.path_nodes = new_path_nodes

        # Set the path
        self.path_wrapper.set_path(self.path_nodes)

        # Set planning as done
        self.planning_done = True

        self.last_valid = None

        # Enable moving obstacles
        self.moving_obstacles_status = True
