from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node
from model.geometry.point import Point


class DepthFirstSearch(SearchBased):

    def __init__(self, map, start=Point(0, 0), boundary=0.2, iterations=1, discretization_step=0.2):
        super().__init__(map, start, boundary, iterations, discretization_step)

    def pre_search(self):
        self.path = []
        self.draw_list = []
        self.generated_neighbors = set()
        self.open_set = [Node(self.start)]  # Use a simple list as a stack

        self.map.disable_moving_obstacles()

    def can_run(self):
        return self.open_set and not self.has_path()

    def step_search(self):

        current_node = self.open_set.pop()
        print(f'Open set: {self.open_set}')

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

    def post_search(self):
        print(f'Before smoothing: {self.path}')
        self.smooth()
        print(f'After smoothing : {self.path}')
        print(f'Has path: {self.has_path()}')

    def heuristic(self, point):
        pass

    def reconstruct_path(self, goal_node):
        # Reconstruct the path by backtracking through the parent pointers
        self.path = []
        current_node = goal_node
        while current_node is not None:
            self.path.insert(0, current_node.point)
            current_node = current_node.parent
