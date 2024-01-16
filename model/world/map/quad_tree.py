from model.geometry.polygon import Polygon
from model.geometry.point import Point

"""
# Graphic
import matplotlib.pyplot as plt
import matplotlib.patches as patches
"""


class QuadTreeNode:
    def __init__(self, bounds):
        self.bounds = bounds
        self.children = [None, None, None, None]  # NW, NE, SW, SE
        self.polygons = []

    def insert(self, polygon_id, polygon):
        if not self.in_bounds(polygon.get_bounds()):
            return False

        if len(self.polygons) < 4:  # If there's space, insert the polygon here
            self.polygons.append((polygon_id, polygon))
            return True

        # If the node is a leaf, split it into four children
        if all(child is None for child in self.children):
            self.split()

        # Recursively insert the polygon into one of the children
        for i in range(4):
            if self.children[i].insert(polygon_id, polygon):
                return True

    def remove(self, polygon_id):
        # Remove the polygon with the specified ID from the node
        self.polygons = [polygon for polygon in self.polygons if polygon[0] != polygon_id]

        """
        for polygon in self.polygons:
            print(f'{polygon}')
        """

        # Recursively remove the polygon from the children
        for child in self.children:
            if child is not None:
                child.remove(polygon_id)

    def query_region(self, query_bounds):
        result = []

        # Check if the node's bounds intersect with the query region
        if not self.in_bounds(query_bounds):
            return result

        # Add IDs of polygons in the node that intersect with the query region
        for polygon_id, polygon in self.polygons:
            if self.intersects(polygon.get_bounds(), query_bounds):
                result.append(polygon_id)

        # Recursively query the children
        for child in self.children:
            if child is not None:
                result.extend(child.query_region(query_bounds))

        return result

    def in_bounds(self, polygon_bounds):
        min_x, min_y, max_x, max_y = self.bounds
        p_min_x, p_min_y, p_max_x, p_max_y = polygon_bounds
        return not (p_max_x < min_x or p_min_x > max_x or p_max_y < min_y or p_min_y > max_y)

    def split(self):
        min_x, min_y, max_x, max_y = self.bounds
        mid_x, mid_y = (min_x + max_x) / 2, (min_y + max_y) / 2

        self.children[0] = QuadTreeNode((mid_x, mid_y, max_x, max_y))  # NW
        self.children[1] = QuadTreeNode((min_x, mid_y, mid_x, max_y))  # NE
        self.children[2] = QuadTreeNode((min_x, min_y, mid_x, mid_y))  # SW
        self.children[3] = QuadTreeNode((mid_x, min_y, max_x, mid_y))  # SE

        # Reallocate polygons to children
        for polygon_id, polygon in self.polygons:
            for child in self.children:
                if child.in_bounds(polygon.get_bounds()):
                    child.insert(polygon_id, polygon)
                    break

        self.polygons = []  # Clear polygons from the current node

    @staticmethod
    def intersects(bounds1, bounds2):
        min_x1, min_y1, max_x1, max_y1 = bounds1
        min_x2, min_y2, max_x2, max_y2 = bounds2

        return not (max_x1 < min_x2 or min_x1 > max_x2 or max_y1 < min_y2 or min_y1 > max_y2)

    """
    def draw(self, ax):
        min_x, min_y, max_x, max_y = self.bounds
        rect = patches.Rectangle((min_x, min_y), max_x - min_x, max_y - min_y, linewidth=1, edgecolor='b',
                                 facecolor='none')
        ax.add_patch(rect)

        for child in self.children:
            if child is not None:
                child.draw(ax)

        for polygon_id, polygon in self.polygons:
            p_min_x, p_min_y, p_max_x, p_max_y = polygon.get_bounds()
            rect = patches.Rectangle((p_min_x, p_min_y), p_max_x - p_min_x, p_max_y - p_min_y, linewidth=1,
                                     edgecolor='r', facecolor='none')
            ax.add_patch(rect)
    """


class QuadTree:

    def __init__(self, bounds):
        self.root = QuadTreeNode(bounds)

    def insert(self, polygon_id, polygon):
        return self.root.insert(polygon_id, polygon)

    def remove(self, polygon_id):
        # Remove the polygon from the quad tree starting from the root
        self.root.remove(polygon_id)

    def query_region(self, query_bounds):
        # Query the quad tree starting from the root
        return self.root.query_region(query_bounds)

    """
    def draw(self):
        fig, ax = plt.subplots()
        ax.set_xlim(self.root.bounds[0], self.root.bounds[2])
        ax.set_ylim(self.root.bounds[1], self.root.bounds[3])
        self.root.draw(ax)
        plt.show()
    """


# Example usage:
if __name__ == "__main__":
    quad_tree_bounds = (0, 0, 100, 100)
    quad_tree = QuadTree(quad_tree_bounds)

    # Example polygons with IDs
    polygons = [
        (1, Polygon([Point(10, 10), Point(10, 30), Point(30, 30), Point(30, 10)])),
        (2, Polygon([Point(40, 40), Point(40, 60), Point(60, 60), Point(60, 40)])),
        (3, Polygon([Point(70, 70), Point(70, 90), Point(90, 90), Point(90, 70)])),
        (4, Polygon([Point(20, 20), Point(20, 80), Point(80, 80), Point(80, 20)]))
    ]

    # Insert polygons into the quad tree
    for polygon_id, polygon_bounds in polygons:
        quad_tree.insert(polygon_id, polygon_bounds)

    # Draw the quad tree
    # quad_tree.draw()

    # Remove a polygon from the quad tree
    quad_tree.remove(2)

    # Query region (50, 50, 80, 80) and print the result
    query_bounds = (50, 50, 80, 80)
    result = quad_tree.query_region(query_bounds)
    print(f'Result after removing idx 2: {result}')
