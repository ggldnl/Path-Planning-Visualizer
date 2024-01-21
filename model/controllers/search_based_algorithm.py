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

    def __init__(self,
                 world_map,
                 start=Point(0, 0),
                 margin=0.2,
                 iterations_per_step=1,
                 dynamic=False,
                 discretization_step=0.2
                 ):

        if world_map.map_boundaries is None:
            raise ValueError('The map should be bounded for search based algorithms')

        # Side of the area that each node covers
        self.discretization_step = discretization_step

        # Open and closed set
        self.open_set = None
        self.closed_set = None

        # Each time we generate the neighbors of a node, we check
        # if the respective node has already been generated in a previous iteration
        # to avoid adding it twice to the open_set
        self.generated_neighbors = set()

        super().__init__(world_map, start, margin, iterations_per_step, dynamic)

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

    def cell_contains(self, center, point):
        """
        Returns True if the cell centered in center and with side self.discretization_step
        contains the point, False otherwise
        """
        return (center.x - self.discretization_step / 2 <= point.x <= center.x + self.discretization_step / 2 and
                center.y - self.discretization_step / 2 <= point.y <= center.y + self.discretization_step / 2)

    def has_path(self):
        return len(self.path) > 0 and self.cell_contains(self.path[-1], self.world_map.goal)

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

                    if (self.world_map.map_boundaries[0] <= neighbor_x <= self.world_map.map_boundaries[2] and
                            self.world_map.map_boundaries[1] <= neighbor_y <= self.world_map.map_boundaries[3]):

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
