from abc import abstractmethod
from model.geometry.segment import Segment


class Controller:

    def __init__(self, robot, map):

        self.robot = robot
        self.map = map

        # Range from the next target point in which the robot must recompute the path
        # (rerun the path planning algorithm)
        self.EPS = 0.1

        # List of points to reach the goal
        self.path = []
        self.idx = 0

    def step(self):

        if (
            not self.is_robot_near_goal() or
            self.is_path_obstructed() or
            len(self.path) == 0
        ):
            self.search()

        next = self.path[self.idx]
        self.idx += 1
        return next

    def is_path_obstructed(self):
        for i in range(1, len(self.path)):
            segment = Segment(self.path[i - 1], self.path[i])
            bbox = segment.bounds
            dangerous_obstacle_ids = self.map.query_region(bbox)
            if len(dangerous_obstacle_ids) > 0:
                return True
        return False

    def is_robot_near_goal(self):
        return self.robot.current_pose.distance(self.map.current_goal) < self.EPS

    @abstractmethod
    def search(self):
        pass
