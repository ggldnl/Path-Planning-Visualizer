from abc import abstractmethod

from model.geometry.pose import Pose
from model.geometry.point import Point
from model.geometry.segment import Segment
from model.geometry.polygon import Polygon

import numpy as np

from model.exceptions.empty_path_exception import EmptyPathException


class Controller:

    def __init__(self, robot, map, iterations, discretization_step):

        self.robot = robot
        self.map = map
        self.iterations = iterations
        self.discretization_step = discretization_step

        # Range from the next target point in which the robot must recompute the path
        # (rerun the path planning algorithm)
        self.EPS = 0.1

        # List of points to reach the goal
        self.path = []

        # Search based algorithms work by discretizing the map into a grid and using nodes.
        # We can define a set containing all the nodes already generated to avoid cluttering
        # the open set of the search based algorithms by adding the same node multiple times.
        # Sampling based algorithms will not use this set
        self.generated_neighbors = set()

        # List of objects that should be drawn on screen. This could be a list of
        # expanded nodes for search-based algorithms or a list of segments representing the
        # branches of a tree for sampling-based algorithms.
        self.draw_list = []

    def step(self):
        """
        Does one step of the search loop, regardless of whether the path is obstructed or something else
        has happened: the actual path planning algorithm will account for this (that is why it is called
        path planning with moving obstacles).
        """
        self.search()

    # TODO add path smoothing to the step routine
    def path_smoothing(self):
        """
        Used to smooth paths to eliminate unnecessary detours
        """
        start = self.path[0]
        idx = 0
        for i in range(len(self.path), 0, -1):
            point = self.path[-1]
            if not self.check_collision(start, point):
                idx = i
                break
        self.path = self.path[0] + self.path[idx:]

    def has_path(self):
        """
        Returns true if the controller has a complete path to the goal.
        The controller has a FULL path when it has a path and the last element of the path is the goal;
        this means the controller can build up the path through multiple step iterations and only when
        the path is ready it is executed by the robot.
        """
        return len(self.path) > 0 and self.path[-1] == self.map.goal

    def next(self):
        """
        Returns next point to reach if there is a FULL path to the goal. If the method is called before
        checking if the path actually exists it raises an EmptyPathException, much like the has_next()
        and next() method works in an iterator.
        """

        if len(self.path) == 0:
            raise EmptyPathException()

        # The robot could take multiple step iterations to reach the goal. That is why we should
        # return the last point until the robot has reached it, and only then remove it from the path.
        # When the path is empty, the robot has reached the target
        current_target = self.path[0]

        # If the robot has reached the point
        if self.is_robot_at(current_target):
            # Remove it from the list
            self.path.pop(0)

        # Add the orientation (keep the orientation of the robot while moving towards the goal)
        current_x, current_y, current_theta = self.robot.current_pose
        new_x, new_y = current_target
        delta_x = new_x - current_x
        delta_y = new_y - current_y
        return Pose(new_x, new_y, np.arctan2(delta_y, delta_x))

    def search(self):
        for _ in range(0, self.iterations):
            self._search()

    @abstractmethod
    def _search(self):
        """
        Single step of the path planning loop.
        """
        raise NotImplementedError

    @abstractmethod
    def _init(self):
        """
        We might want to initialize other algorithm-specific data structures too.
        """
        raise NotImplementedError

    def reset(self):
        """
        Reset the controller
        """
        self.path = []
        self.generated_neighbors = set()
        self.draw_list = []

        # Algorithm specific reset
        self._init()

    def get_neighbors(self, point, include_current=False):

        # The point might not be exactly a vertex of a grid with size discretization_step
        new_x = round(point.x / self.discretization_step) * self.discretization_step
        new_y = round(point.y / self.discretization_step) * self.discretization_step

        neighbors = []

        for i in range(-1, 2):
            for j in range(-1, 2):
                if not (i == 0 and j == 0) or include_current:

                    # Generate neighbor coordinates
                    neighbor_x = new_x + i * self.discretization_step
                    neighbor_y = new_y + j * self.discretization_step

                    # Round them to match the grid
                    neighbor_x = round(neighbor_x / self.discretization_step) * self.discretization_step
                    neighbor_y = round(neighbor_y / self.discretization_step) * self.discretization_step

                    neighbor = Point(neighbor_x, neighbor_y)

                    if neighbor in self.generated_neighbors:
                        continue

                    if self.check_collision(point, neighbor):
                        continue

                    neighbors.append(neighbor)

        return neighbors

    def check_collision(self, start, end):
        """
        Check if there is an obstacle between the two points
        """
        line = Segment(start, end)
        buffer = Polygon.get_segment_buffer(line, left_margin=self.discretization_step, right_margin=self.discretization_step)
        intersecting_obstacles_ids = self.map.query_region(buffer)
        return len(intersecting_obstacles_ids) > 0

    def is_robot_near_goal(self):
        return self.robot.current_pose.distance(self.map.goal) < self.EPS

    def is_robot_at_goal(self):
        return self.robot.current_pose.as_point() == self.map.goal

    def is_robot_at(self, point):
        return self.robot.current_pose.as_point() == point
