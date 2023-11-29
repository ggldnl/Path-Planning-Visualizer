from queue import PriorityQueue

from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node

from model.geometry.point import Point


class BestFirstSearch(SearchBased):

    def __init__(self, map, start=Point(0, 0), boundary=0.2):
        super().__init__(map, start, boundary)

    def init(self):
        self.path = []
        self.draw_list = []
        self.generated_neighbors = set()
        self.open_set = PriorityQueue()
        self.open_set.put((0, Node(self.start)))

        self.map.disable_moving_obstacles()

    def heuristic(self, point):
        return point.distance(self.map.goal)

    def step(self):

        if not self.open_set.empty():

            _, current_node = self.open_set.get()

            if current_node.point == self.map.goal:
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
        while current_node is not None:
            path.append(current_node.point)
            current_node = current_node.parent

        self.path = path[::-1]
