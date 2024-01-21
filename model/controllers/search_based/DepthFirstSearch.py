from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node
from model.geometry.point import Point


class DepthFirstSearch(SearchBased):

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 margin=0.2,
                 iterations_per_step=1,
                 discretization_step=0.2
                 ):

        super().__init__(
            world_map,
            start,
            margin=margin,
            iterations_per_step=iterations_per_step,
            dynamic=False,
            discretization_step=discretization_step
        )

    def pre_search(self):
        self.path = []
        self.draw_list = []
        self.generated_neighbors = set()
        self.open_set = [Node(self.start)]  # Use a simple list as a stack

    def can_run(self):
        return self.open_set and not self.has_path()

    def step_search(self):

        current_node = self.open_set.pop(-1)

        if self.cell_contains(current_node.point, self.world_map.goal):
            # Goal reached, reconstruct the path
            self.reconstruct_path(current_node)
            return

        # Expand the current node and add its neighbors to the frontier
        neighbors = self.get_neighbors(current_node.point)
        for neighbor in neighbors:
            new_node = Node(neighbor, current_node)
            self.open_set.append(new_node)

            # Update the draw_list
            self.draw_list.append(self.get_view(neighbor))

    # Uncomment to enable smoothing
    """
    def post_search(self):
        # self.smooth()
        return 0

    def check_collision(self, start, end):
        if start == end:
            return False
        return super().check_collision(start, end)
    """

    def heuristic(self, point):
        pass

    def reconstruct_path(self, goal_node):
        # Reconstruct the path by backtracking through the parent pointers
        self.path = []
        current_node = goal_node

        # Change the point from the center of the cell that contains the goal to the goal itself
        current_node.point = self.world_map.goal

        while current_node is not None:
            self.path.insert(0, current_node.point)
            current_node = current_node.parent
