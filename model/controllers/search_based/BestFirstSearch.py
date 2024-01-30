from queue import PriorityQueue

from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node

from model.geometry.point import Point


class BestFirstSearch(SearchBased):

    def __init__(self, 
                 world_map, 
                 start=Point(0, 0), 
                 margin=0.2, 
                 iterations_per_step=1,
                 max_iterations=5000,
                 discretization_step=0.2
                 ):
        
        super().__init__(
            world_map,
            start,
            margin=margin,
            iterations_per_step=iterations_per_step,
            max_iterations=max_iterations,
            dynamic=False,
            discretization_step=discretization_step
        )

    def pre_search(self):
        self.open_set = PriorityQueue()
        self.open_set.put((0, Node(self.start)))

    def heuristic(self, point):
        return point.distance(self.world_map.goal)

    def can_run(self):
        return not self.open_set.empty() and not self.has_path()

    def step_search(self):

        _, current_node = self.open_set.get()

        if self.cell_contains(current_node.point, self.world_map.goal):
            # Goal reached, reconstruct the path
            self.reconstruct_path(current_node)
            return

        # Expand the current node and add its neighbors to the frontier
        neighbors = self.get_neighbors(current_node.point)
        for neighbor in neighbors:
            new_node = Node(neighbor, parent=current_node)
            priority = self.heuristic(neighbor)
            self.open_set.put((priority, new_node))

            # Update the draw_list
            self.draw_list.append(self.get_view(neighbor))

    def reconstruct_path(self, goal_node):
        """
        Reconstruct the path by backtracking through the parent pointers
        """
        path = []
        current_node = goal_node

        # Change the point from the center of the cell that contains the goal to the goal itself
        current_node.point = self.world_map.goal

        while current_node is not None:
            path.append(current_node.point)
            current_node = current_node.parent

        self.path = path[::-1]
