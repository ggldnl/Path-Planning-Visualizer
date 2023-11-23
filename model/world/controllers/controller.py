from abc import abstractmethod

from model.geometry.pose import Pose

import numpy as np

from model.exceptions.empty_path_exception import EmptyPathException


class Controller:

    def __init__(self, robot, map):

        self.robot = robot
        self.map = map

        # Range from the next target point in which the robot must recompute the path
        # (rerun the path planning algorithm)
        self.EPS = 0.1

        # List of points to reach the goal
        self.path = []

    def step(self):

        if (len(self.path) == 0 and not self.is_robot_near_goal()):
                #or self.is_path_obstructed()):
            print('Searching...')
            self.search()
            print(f'Path found! {self.path}')

    @abstractmethod
    def get_draw_list(self):
        """
        Add to the draw_list everything that should be drawn on screen. This can be the path (A*) or the entire
        tree (RRT, RRT*, ...)
        """
        pass

    def next(self):
        """
        Returns next point to reach or None, as it should be called repeatedly even if no new data is available
        """

        if len(self.path) == 0:
            # raise EmptyPathException()
            return None

        current_target = self.path[-1]

        # If the robot has reached the point
        if self.is_robot_at(current_target):

            # Remove it from the list
            self.path.pop()

        current_x, current_y, current_theta = self.robot.current_pose
        new_x, new_y = current_target
        delta_x = new_x - current_x
        delta_y = new_y - current_y
        return Pose(new_x, new_y, np.arctan2(delta_y, delta_x))

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

    @abstractmethod
    def search(self):
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """
        We may want to clean other structures too
        """
        raise NotImplementedError
