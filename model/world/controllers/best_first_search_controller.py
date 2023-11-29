from model.world.controllers.controller import Controller
from model.geometry.point import Point
from model.geometry.polygon import Polygon

from queue import PriorityQueue


class Node:

    def __init__(self, point, cost, heuristic, parent=None):
        self.point = point
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    @property
    def x(self):
        return self.point.x

    @property
    def y(self):
        return self.point.y

    def __str__(self):
        return f'Node({self.point}, cost={self.cost:.2f}, heuristic={self.heuristic:.2f})'

    def __repr__(self):
        return self.__str__()

    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost + other.heuristic


class BestFirstSearchController(Controller):

    def __init__(self, robot, map, iterations=1, discretization_step=0.2):

        super().__init__(robot, map, iterations, discretization_step)

        # Closed set
        self.visited = set()

        # Open set
        self.priority_queue = PriorityQueue()

        # Start point
        self.start = None

        self._init()

    def _init(self):

        self.visited = set()
        self.priority_queue = PriorityQueue()

        start_point = self.robot.current_pose.as_point()
        self.start = Node(start_point, cost=0, heuristic=self.heuristic_function(start_point))

        self.priority_queue.put(self.start)

        # Disable moving obstacles for the map
        self.map.disable_moving_obstacles()

    @staticmethod
    def heuristic_function(point):
        return 0

    def reconstruct_path(self, node):

        self.path = [Point(self.map.goal[0], self.map.goal[1])]
        node_now = node

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.x, node_now.y))

        self.path = self.path[::-1]

    def _search(self):

        if not self.has_path() and not self.priority_queue.empty():

            current_node = self.priority_queue.get()
            current_point = current_node.point

            if current_point == self.map.goal:
                return self.reconstruct_path(current_node)

            self.visited.add(current_point)

            for neighbor in self.get_neighbors(current_point):

                self.generated_neighbors.add(neighbor)

                if neighbor not in self.visited:
                    step_cost = current_point.distance(neighbor)
                    neighbor_node = Node(
                        neighbor,
                        cost=current_node.cost + step_cost,
                        heuristic=self.heuristic_function(neighbor),
                        parent=current_node
                    )

                    self.priority_queue.put(neighbor_node)
                    self.visited.add(neighbor)

                    # The draw_list for A* will contain all the points in the open_set
                    tile = Polygon([
                        Point(neighbor.x - self.discretization_step / 2,
                              neighbor.y + self.discretization_step / 2),
                        Point(neighbor.x - self.discretization_step / 2,
                              neighbor.y - self.discretization_step / 2),
                        Point(neighbor.x + self.discretization_step / 2,
                              neighbor.y - self.discretization_step / 2),
                        Point(neighbor.x + self.discretization_step / 2,
                              neighbor.y + self.discretization_step / 2),
                    ])
                    self.draw_list.append(tile)
