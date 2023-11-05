from model.geometry.point import Point


class Segment:

    def __init__(self, start, end):
        self.start = Point(start[0], start[1])
        self.end = Point(end[0], end[1])

    @property
    def bounds(self):
        min_x = min(self.start.x, self.end.x)
        max_x = max(self.start.x, self.end.x)
        min_y = min(self.start.y, self.end.y)
        max_y = max(self.start.y, self.end.y)
        return (min_x, min_y, max_x, max_y)

    def intersects(self, polygon):
        # Get the edges of the polygon
        edges_polygon = polygon.get_edges()

        for edge_polygon in edges_polygon:
            axis = self._normal()  # Calculate the normal vector of the segment
            min1, max1 = self._project(axis)
            min2, max2 = edge_polygon._project(axis)

            if max1 < min2 or max2 < min1:
                # If there is a gap along this axis, the segment and polygon do not intersect
                return False

        return True

    def _normal(self):
        # Compute the normal vector of the segment
        return Point(-(self.end.y - self.start.y), self.end.x - self.start.x)

    def _project(self, axis):
        # Project the segment onto an axis and return the min and max values
        min_proj = min(self.start.x * axis.x + self.start.y * axis.y,
                       self.end.x * axis.x + self.end.y * axis.y)
        max_proj = max(self.start.x * axis.x + self.start.y * axis.y,
                       self.end.x * axis.x + self.end.y * axis.y)
        return min_proj, max_proj
