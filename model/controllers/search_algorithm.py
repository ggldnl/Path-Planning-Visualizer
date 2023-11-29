from abc import ABC, abstractmethod

from model.geometry.segment import Segment
from model.geometry.polygon import Polygon


class SearchAlgorithm(ABC):
    """
    This interface will represent all the search algorithms. Every algorithm has an initial
    phase in which data structures are set up and a search loop in which algorithm-specific
    logic is repeatedly executed.

    sa.init()
    while not sa.has_terminated():
        sa.step()

    """

    def __init__(self, map, start, boundary):

        # Map
        self.map = map

        # Start point
        self.start = start

        # The path should be distant from each obstacle by at least self.boundary/2
        self.boundary = boundary

        # List of points from start (first) to goal (last)
        self.path = []

        # List of objects that should be drawn on screen. This could be a list of
        # expanded nodes for search-based algorithms or a list of segments representing the
        # branches of a tree for sampling-based algorithms.
        self.draw_list = []

        self.init()

    @abstractmethod
    def init(self):
        """
        Init the search algorithm by instantiating all the necessary data structures (at creation/reset time).
        """
        pass

    def reset(self):
        """
        Reset the search algorithm (alias for init).
        """
        self.init()

    @abstractmethod
    def step(self):
        """
        Search step. Our world has an update loop in which the state of the system is advanced.
        We leverage this (outer) loop to call the step() method of a controller. A controller
        (check Controller class) has a search algorithm and is responsible for:
        1. make progress computing the path one step at a time
        2. talk to the robot by streaming the path (if found) or by making it hold its position
        The controller can make one or more (to increase convergence speed) steps of the search
        algorithm at a time.
        """
        pass

    def smooth(self):
        """
        Used to smooth the path and eliminate unnecessary detours.
        """
        start = self.path[0]
        idx = 0
        for i in range(len(self.path), 0, -1):
            point = self.path[-1]
            if not self.check_collision(start, point):
                idx = i
                break
        self.path = self.path[0] + self.path[idx:]

    def check_collision(self, start, end):
        """
        Given two points on the map, this implements the logic with which we check if
        the second point is reachable by the first
        """
        line = Segment(start, end)
        buffer = Polygon.get_segment_buffer(line, left_margin=self.boundary/2, right_margin=self.boundary/2)
        intersecting_obstacles_ids = self.map.query_region(buffer)
        return len(intersecting_obstacles_ids) > 0

    def has_terminated(self):
        """
        Termination condition: if the path contains points and the last point is the goal,
        then the path is complete and the robot can follow it.
        """
        return len(self.path) > 0 and self.path[-1] == self.map.goal
