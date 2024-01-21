from model.controllers.sampling_based_algorithm import SamplingBased
from model.controllers.graph import Node

from model.geometry.point import Point

import numpy as np


class RRT(SamplingBased):

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 margin=0.2,
                 iterations_per_step=1,
                 max_iterations=8000,
                 step_length=0.2,
                 goal_sample_rate=0.05,
                 ):

        self.step_length = step_length
        self.node_new = None
        self.new_node_to_goal_dist = 0

        super().__init__(
            world_map,
            start,
            margin=margin,
            iterations_per_step=iterations_per_step,
            max_iterations=max_iterations,
            dynamic=False,
            goal_sample_rate=goal_sample_rate
        )

    def heuristic(self, point):
        return 0

    def pre_search(self):

        self.node_new = Node(self.start)
        self.nodes = [self.node_new]
        self.new_node_to_goal_dist = self.node_new.point.distance(self.world_map.goal)

        self.edges = []
        self.current_iteration = 0

        self.path = []
        self.draw_list = []

    def step_search(self):

        self.current_iteration += 1

        node_rand = self.generate_random_node()
        node_near = self.nearest_neighbor(node_rand)
        self.node_new = self.new_state(node_near, node_rand)

        if self.node_new and not self.check_collision(node_near.point, self.node_new.point):
            self.nodes.append(self.node_new)
            self.new_node_to_goal_dist = self.distance_to_goal(self.node_new)

            # Update drawing list
            self.update_draw_list()

    def can_run(self):
        """
        Algorithm terminates when:
         1. the goal is found;
         2. we exceed the maximum number of iterations;
         3. the distance of the newly generated node from the goal is less than the step length;
        Algorithm can thus run until:
         1. the goal has not been found yet (we don't have a full path);
         2. we have iterations left;
         3. the distance of the newly generated node from the goal is greater than the step length;

        We don't have memory constraints, but we have time constraints specified in
        number of iterations.
        """
        return (
                not self.has_path() and
                self.current_iteration < self.max_iterations and
                self.new_node_to_goal_dist > self.step_length
        )

    def post_search(self):
        self.extract_path(self.node_new)

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

    def extract_path(self, node):

        self.path = [self.world_map.goal]
        node_now = node

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.point.x, node_now.point.y))

        self.path = self.path[::-1]

    """
    def update_draw_list(self, node):
        child_point = node.point
        parent_point = node.parent.point if node.parent is not None else None
        self.draw_list.append(child_point)
        if parent_point is not None:
            self.draw_list.append(Segment(parent_point, child_point))
    """