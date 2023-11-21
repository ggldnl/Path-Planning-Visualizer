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
        return min_x, min_y, max_x, max_y

    def get_edges(self):
        return [self.copy()]

    def copy(self):
        return Segment(self.start.copy(), self.end.copy())

    def __getitem__(self, item):
        if item == 0:
            return self.start
        elif item == 1:
            return self.end
        raise IndexError(f'Segment index out of range: {item}')

    def __setitem__(self, key, value):

        if not isinstance(value, Point):
            key_name = 'start' if key == 0 else 'end'
            raise ValueError(f'Invalid {key_name} object {type(value)}; must be Point.')

        if key == 0:
            self.start = value
        elif key == 1:
            self.end = value
        raise IndexError(f'Segment index out of range: {key}')

    def __iter__(self):
        return iter([self.start, self.end])

    def normal(self):
        """
        Compute the normal vector of the segment
        """
        return Point(-(self.end.y - self.start.y), self.end.x - self.start.x)

    def project(self, axis):
        """
        Project the segment onto an axis and return the min and max values
        """

        min_proj = min(self.start.x * axis.x + self.start.y * axis.y,
                       self.end.x * axis.x + self.end.y * axis.y)
        max_proj = max(self.start.x * axis.x + self.start.y * axis.y,
                       self.end.x * axis.x + self.end.y * axis.y)
        return min_proj, max_proj

    def closest_point(self, point):
        """
        Find the closest point on the segment to the given point
        """

        x1, y1 = self.start.x, self.start.y
        x2, y2 = self.end.x, self.end.y
        x, y = point.x, point.y

        dx = x2 - x1
        dy = y2 - y1

        t = ((x - x1) * dx + (y - y1) * dy) / (dx**2 + dy**2)

        if t < 0:
            return self.start
        elif t > 1:
            return self.end
        else:
            return Point(x1 + t * dx, y1 + t * dy)
