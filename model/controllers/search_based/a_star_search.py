from queue import PriorityQueue

from model.geometry.point import Point

from model.controllers.search_based_algorithm import SearchBased
from model.controllers.graph import Node


class AStarSearch(SearchBased):

    def __init__(self, map, start=Point(0, 0), boundary=0.2):
        super().__init__(map, start, boundary)

    def init(self, ):

        self.path = []
        self.draw_list = []
        self.generated_neighbors = set()

        self.open_set = PriorityQueue()

        start_node = Node(self.start, cost=0, heuristic=self.heuristic(self.start))

        self.open_set.put((start_node.cost + start_node.heuristic, start_node))  # Priority queue with f(n) as priority

        self.map.disable_moving_obstacles()

    def heuristic(self, point):
        return point.distance(self.map.goal)

    def step(self):
        if not self.open_set.empty():
            current_node = self.open_set.get()[1]

            if current_node.point == self.map.goal:
                # Goal reached, reconstruct the path
                self.reconstruct_path(current_node)
                return

            # Expand the current node and add its neighbors to the frontier
            neighbors = self.get_neighbors(current_node.point)
            for neighbor in neighbors:
                new_cost = current_node.cost + current_node.point.distance(neighbor)
                new_heuristic = self.heuristic(neighbor)
                new_node = Node(neighbor, parent=current_node, cost=new_cost, heuristic=new_heuristic)
                priority = new_cost + new_heuristic
                self.open_set.put((priority, new_node))

                # Update draw list
                self.draw_list.append(self.get_view(neighbor))

    def reconstruct_path(self, goal_node):
        # Reconstruct the path by backtracking through the parent pointers
        self.path = []
        current_node = goal_node
        while current_node is not None:
            self.path.insert(0, current_node.point)
            current_node = current_node.parent
