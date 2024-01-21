import random
from abc import ABC, abstractmethod

from model.geometry.segment import Segment
from model.geometry.polygon import Polygon


class SearchAlgorithm(ABC):
    """
    This interface represents all the search algorithms. Every algorithm has an initial
    phase in which data structures are set up, a search loop in which algorithm-specific
    logic is repeatedly executed to gather necessary information and a post loop phase
    in which the gathered information is processed in order to produce the path.

    sa = ConcreteSearchAlgorithm() (implements SearchAlgorithm, calls init to set up data structures)

    sa.pre_search()
    while sa.can_run():
        sa.step_search()
    sa.post_search()
    """

    def __init__(self, world_map, start, margin, iterations_per_step=1, dynamic=False):

        # Map
        self.world_map = world_map

        # Start point
        self.start = start

        # The path should be distant from each obstacle by at least self.margin/2
        self.margin = margin

        # Number of steps to execute each time (default = 1)
        self.iterations_per_step = iterations_per_step

        # List of points from start (first) to goal (last)
        self.path = []

        # List of objects that should be drawn on screen. This could be a list of
        # expanded nodes for search-based algorithms or a list of segments representing the
        # branches of a tree for sampling-based algorithms.
        self.draw_list = []

        # State variables
        self.post_search_performed = False

        # Algorithms that work in dynamic environments should respond
        # to changes in the map, other algorithms does not have this
        # requirement. We can disable changes in the map during the
        # search step and enable them only before the search starts
        self.dynamic = dynamic
        self.map_changes_enabled = True
        self.world_map.enable()

        # Perform the pre-search steps
        self.pre_search()

    def reset(self):
        """
        Reset the search algorithm (alias for init).
        """

        self.map_changes_enabled = True
        self.world_map.enable()

        self.post_search_performed = False
        self.pre_search()

    def smooth(self):
        """
        Used to smooth the path and eliminate unnecessary detours.
        """
        start = self.path[0]
        idx = 0
        for i in range(len(self.path) - 1, 0, -1):
            point = self.path[i]
            if not self.check_collision(start, point):
                idx = i
                break
        self.path = [self.path[0]] + self.path[idx:]

    def check_collision(self, start, end):
        """
        Given two points on the map, this implements the logic with which we check if
        the second point is reachable by the first
        """
        line = Segment(start, end)
        buffer = Polygon.segment_buffer(line, left_margin=self.margin/2, right_margin=self.margin/2)
        intersecting_obstacles_ids = self.world_map.query_polygon(buffer)
        return len(intersecting_obstacles_ids) > 0

    def has_path(self):
        """
        Return True if the algorithm has found a path. A path is a list of points
        that starts from the robot position and ends with the goal.
        If the path contains points and the last point is the goal, then the path
        is complete and the robot can follow it.
        """
        return len(self.path) > 0 and self.path[-1] == self.world_map.goal

    def has_terminated(self):
        """
        True if the algorithm has terminated. In its basic form an algorithm
        is finished when it cannot run anymore. This does not take into account
        the fact that the algorith has found the path or not.
        """
        return not self.can_run()

    @abstractmethod
    def can_run(self):
        """
        Condition until which the search loop can go on. In some cases the search
        loop will end when the first path is found (condition = path found or no
        more resources) and in some other cases we should go on until termination
        condition (the more we go on, the better is the outcome, like in RRT*).
        """
        pass

    def pre_search(self):
        """
        Init the search algorithm by instantiating all the necessary data structures
        (at creation/reset time) and make initial steps. This routine is called also
        when the reset method is invoked.
        """
        pass

    def post_search(self):
        """
        Steps to be performed after the search loop. Some algorithms have nothing to do
        after the search loop, some others has to reconstruct the path, ...
        """
        return

    @abstractmethod
    def step_search(self):
        """
        Search step. Our world has an update loop in which the state of the system is advanced.
        We leverage this (outer) loop to call the step() method of a controller. A controller
        (check Controller class) has a search algorithm and is responsible for:
        1. making progress computing the path one step at a time;
        2. talking to the robot by streaming the path (if found) or by making it hold its position;
        3. drawing on screen insights on what the search algorithm is doing;
        """
        pass

    def step(self):
        """
        Step the search algorithm. The step is performed even if there is nothing more to do.
        The initial step is performed at construction/reset time. This method calls the step_search
        if the search loop has not ended yet and the post search only once if the loop has ended
        and the post search has not been executed.
        """

        if not self.dynamic:
            self.map_changes_enabled = False
            self.world_map.disable()

        for _ in range(self.iterations_per_step):

            # If the algorithm has not yet terminated (while search time/space remaining)
            if self.can_run():

                # Progress the search
                self.step_search()

                # Eventually, step_search will set the step_search_done flag to True
                # and the step loop will end -> post_search

            # Else, if the algorithm has terminated
            else:

                # Perform final steps only if we haven't exceeded time constraints
                if not self.post_search_performed:
                    self.post_search()
                    self.post_search_performed = True

        # At this point we either have a path or an empty list


class TestSearchAlgorithm(SearchAlgorithm):
    """
    This test class aims to simulate the implementation of the SearchAlgorithm interface.
    It goes on for #fake_iterations_per_step iterations_per_step and then draw a number in range [0, 1) at random:
    if the number is < 0.2, it sets a path.
    The termination condition is to find a path or to exit on time constraint violation.
    """

    def __init__(self, world_map, start, margin=0.2, iterations_per_step=1, max_iterations=10):

        self.max_iterations = max_iterations
        self.current_iteration = 0
        self.goal_found = False

        super().__init__(world_map, start, margin, iterations_per_step)

    def can_run(self):
        # Termination condition = goal found or times up
        return not self.goal_found and self.current_iteration < self.max_iterations

    def pre_search(self):
        print('Pre search')

    def step_search(self):

        print('Searching...')
        self.current_iteration += 1

        n = random.random()
        if n < 0.2:
            print(f'Path found at iteration [{self.current_iteration}]')
            self.path = [None]
            self.goal_found = True

    def post_search(self):
        print('Post search')


if __name__ == '__main__':

    test_search_algorithm = TestSearchAlgorithm(None, None)

    iteration = 0
    max_iterations = 10

    # Infinite loop
    while iteration <= max_iterations:

        test_search_algorithm.step()

        """
        if test_search_algorithm.has_terminated():
            break
        """

        iteration += 1
