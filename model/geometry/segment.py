from model.geometry.point import Point


class Segment:

    def __init__(self, start, end):
        self.start = start
        self.end = end

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
