from abc import abstractmethod

from model.geometry.pose import Pose

import numpy as np

from model.exceptions.empty_path_exception import EmptyPathException


class Controller:

    def __init__(self, robot, map, iterations):

        self.robot = robot
        self.map = map
        self.iterations = iterations

        # Range from the next target point in which the robot must recompute the path
        # (rerun the path planning algorithm)
        self.EPS = 0.1

        # List of points to reach the goal
        self.path = []

    def step(self):
        """
        Does one step of the search loop, regardless of whether the path is obstructed or something else
        has happened: the actual path planning algorithm will account for this (that is why it is called
        path planning with moving obstacles).
        """
        self.search()

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
    def reset(self):
        """
        We might want to clean other structures too.
        """
        raise NotImplementedError

    @abstractmethod
    def get_draw_list(self):
        """
        Return a list of objects that should be drawn on screen. This could be a list of
        expanded nodes for search-based algorithms or a list of segments representing the
        branches of a tree for sampling-based algorithms.
        """
        pass

    def is_path_obstructed(self):
        for i in range(1, len(self.path)):
            if self.map.check_collision(self.path[i - 1], self.path[i]):
                return True
        return False

    def is_robot_near_goal(self):
        return self.robot.current_pose.distance(self.map.goal) < self.EPS

    def is_robot_at_goal(self):
        return self.robot.current_pose.as_point() == self.map.goal

    def is_robot_at(self, point):
        return self.robot.current_pose.as_point() == point
