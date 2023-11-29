from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node
from model.geometry.point import Point


class BreadthFirstSearch(SearchBased):

    def __init__(self, map, start=Point(0, 0), boundary=0.2, discretization_step=0.2):
        super().__init__(map, start, boundary, discretization_step)

    def init(self):
        self.path = []
        self.draw_list = []
        self.generated_neighbors = set()
        self.open_set = [Node(self.start)]  # Use a simple list as a queue

        self.map.disable_moving_obstacles()

    def step(self):
        if self.open_set:
            current_node = self.open_set.pop(0)

            if current_node.point == self.map.goal:
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

    def heuristic(self, point):
        pass

    def reconstruct_path(self, goal_node):
        # Reconstruct the path by backtracking through the parent pointers
        self.path = []
        current_node = goal_node
        while current_node is not None:
            self.path.insert(0, current_node.point)
            current_node = current_node.parent
