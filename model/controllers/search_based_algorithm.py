from abc import abstractmethod

from model.controllers.search_algorithm import SearchAlgorithm
from model.geometry.polygon import Polygon

from model.geometry.point import Point


class SearchBased(SearchAlgorithm):
    """
    This interface represents search based algorithms. Search based algorithms
    explore the state space that we represent as the discretized version of the
    map. The map is thus composed by nodes that covers an area specified by the
    discretization_step parameter.
    Furthermore, we need to keep track of the nodes that are visited but not
    yet expanded and avoid adding them multiple times to the open_set
    """

    def __init__(self, map, start=Point(0, 0), boundary=0.2, iterations=1, discretization_step=0.2):

        # Side of the area that each node covers
        self.discretization_step = discretization_step

        # Open and closed set
        self.open_set = None
        self.closed_set = None

        # Each time we generate the neighbors of a node, we check
        # if the respective node has already been generated in a previous iteration
        # to avoid adding it twice to the open_set
        self.generated_neighbors = set()

        super().__init__(map, start, boundary, iterations)

    def get_view(self, point):
        tile = Polygon([
            Point(point.x - self.discretization_step / 2,
                  point.y + self.discretization_step / 2),
            Point(point.x - self.discretization_step / 2,
                  point.y - self.discretization_step / 2),
            Point(point.x + self.discretization_step / 2,
                  point.y - self.discretization_step / 2),
            Point(point.x + self.discretization_step / 2,
                  point.y + self.discretization_step / 2),
        ])
        return tile

    @abstractmethod
    def heuristic(self, point):
        pass

    def get_neighbors(self, point, include_current=False):

        # The point might not be exactly a vertex of a grid with size discretization_step
        new_x = round(point.x / self.discretization_step) * self.discretization_step
        new_y = round(point.y / self.discretization_step) * self.discretization_step

        neighbors = []

        for i in range(-1, 2):
            for j in range(-1, 2):
                if not (i == 0 and j == 0) or include_current:

                    # Generate neighbor coordinates
                    neighbor_x = new_x + i * self.discretization_step
                    neighbor_y = new_y + j * self.discretization_step

                    if (-self.map.goal_max_dist <= neighbor_x <= self.map.goal_max_dist and
                            -self.map.goal_max_dist <= neighbor_y <= self.map.goal_max_dist):

                        # Round them to match the grid
                        neighbor_x = round(neighbor_x / self.discretization_step) * self.discretization_step
                        neighbor_y = round(neighbor_y / self.discretization_step) * self.discretization_step

                        neighbor = Point(neighbor_x, neighbor_y)

                        # Check if the node has previously been generated
                        if neighbor in self.generated_neighbors:
                            continue

                        # Check if there is a collision
                        if self.check_collision(point, neighbor):
                            continue

                        # Add to the list of nodes to return
                        neighbors.append(neighbor)

                        # Add to the generated_node set so that the next iteration will discard it
                        self.generated_neighbors.add(neighbor)

        return neighbors
