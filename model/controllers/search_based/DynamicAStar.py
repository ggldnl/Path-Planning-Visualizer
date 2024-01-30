import heapq

from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node

from model.geometry.point import Point


class DynamicAStar(SearchBased):

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 margin=0.2,
                 iterations_per_step=1,
                 max_iterations=5000,
                 discretization_step=0.2,
                 ):

        super().__init__(
            world_map,
            start,
            margin=margin,
            iterations_per_step=iterations_per_step,
            max_iterations=max_iterations,
            dynamic=True,
            discretization_step=discretization_step
        )

    def pre_search(self):
        """
        Operations to be executed before the search loop begins
        """
        pass

    def heuristic(self, point):
        return point.distance(self.world_map.goal)

    def can_run(self):
        # Termination condition is that the highest priority element (nearest to the goal) is the goal itself
        # or that we have no more iterations left
        return (
                not self.open_set.empty() and
                not self.cell_contains(self.open_set.queue[0][1].point, self.world_map.goal) and
                self.current_iteration < self.max_iterations
        )

    def step_search(self):
        """
        Search step to be executed inside the search loop
        """
        pass

    def post_search(self):
        """
        Operations to be executed after the search loop ends
        """
        pass
