from model.geometry.shape import Shape
from model.geometry.point import Point


class Circle(Shape):

    def __init__(self, x, y, radius):
        super().__init__()

        self.pose.x = x
        self.pose.y = y
        self.radius = radius

    @property
    def bounds(self):
        return (self.pose.x - self.radius,
                self.pose.y - self.radius,
                self.pose.x + self.radius,
                self.pose.y + self.radius)

    @classmethod
    def get_segment_buffer(cls, segment, margin=None):
        center = segment.midpoint()
        if margin is None:
            margin = len(segment) / 2
        return Circle(center.x, center.y, margin)

    def project(self, axis):
        center_proj = self.pose.x * axis.x + self.pose.y * axis.y
        return center_proj + self.radius, center_proj - self.radius

    def is_inside(self, point):
        distance_squared = (point.x - self.pose.x) ** 2 + (point.y - self.pose.y) ** 2
        return distance_squared <= self.radius ** 2

    def get_center(self):
        return Point(self.pose.x, self.pose.y)

    def __str__(self):
        return f'Circle(center={self.pose.as_point()}, radius={self.radius})'

    def __repr__(self):
        return self.__str__()
