import numpy as np

from model.exceptions.empty_path_exception import EmptyPathException
from model.geometry.pose import Pose


class Controller:
    """
    The controller will be used like this:

    ro = Robot(...)
    sa = SearchAlgorithm(...)
    co = Controller(robot, sa)

    # Scan the map, compute path, ...
    co.step()

    if co.has_path():

        # Get the next target point (could be the same as before if the robot hasn't reached it yet)
        next = co.next()
        ro.target_pose = next

        # Update the robot making it follow the path
        ro.step_motion(dt)
    """

    def __init__(self, robot, search_algorithm, iterations=1):

        self.robot = robot
        self.search_algorithm = search_algorithm
        self.iterations = iterations

    def step(self):

        # Step the search
        self.search_algorithm.step()

        # Handle the next pose
        current_x, current_y, current_theta = self.robot.current_pose
        target_x, target_y, target_theta = self.robot.current_pose

        if self.search_algorithm.has_path() and not self.is_robot_at_goal():

            # Attempt to take the first available point in the path
            current_target = self.search_algorithm.path[0]

            # If the robot has reached the point
            if self.is_robot_at(current_target):
                # Remove it from the list
                self.search_algorithm.path.pop(0)

            target_x = current_target.x
            target_y = current_target.y

        delta_x = target_x - current_x
        delta_y = target_y - current_y

        # Add the orientation (keep the orientation of the robot while moving towards the goal)
        return Pose(target_x, target_y, np.arctan2(delta_y, delta_x))

    def next(self):
        """
        Returns next point to reach if there is a FULL path to the goal. If the method is called before
        checking if the path actually exists it raises an EmptyPathException, much like the has_next()
        and next() method works in an iterator. The returned point can be:
        1. the current position of the robot, if it should be kept steady
        2. a point on the path.
        When a point on the path is returned, the robot starts to walk towards it. It will likely take
        multiple iterations for the robot to reach the target specified position; until then, the
        controller will return the same target point so that the objective of the robot does not change.
        """

        if len(self.search_algorithm.path) == 0:
            raise EmptyPathException()

        # The robot could take multiple step iterations to reach the goal. That is why we should
        # return the last point until the robot has reached it, and only then remove it from the path.
        # When the path is empty, the robot has reached the target
        current_target = self.search_algorithm.path[0]

        # If the robot has reached the point
        if self.is_robot_at(current_target):
            # Remove it from the list
            self.search_algorithm.path.pop(0)

        # Add the orientation (keep the orientation of the robot while moving towards the goal)
        current_x, current_y, current_theta = self.robot.current_pose
        new_x, new_y = current_target
        delta_x = new_x - current_x
        delta_y = new_y - current_y
        return Pose(new_x, new_y, np.arctan2(delta_y, delta_x))

    def is_robot_at(self, point):
        return self.robot.current_pose.as_point() == point

    def is_robot_at_goal(self):
        return self.robot.current_pose.as_point() == self.search_algorithm.world_map.goal

    """
    def has_path(self):
        return self.search_algorithm.has_terminated() and len(self.search_algorithm.path) > 0
    """

    def reset(self):
        self.search_algorithm.start = self.robot.current_pose.as_point()
        self.search_algorithm.reset()
