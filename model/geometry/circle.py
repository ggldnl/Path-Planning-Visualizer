from model.geometry.shape import Shape
from model.geometry.point import Point


class Circle(Shape):

    def __init__(self, x, y, radius):
        super().__init__()

        self.pose.x = x
        self.pose.y = y
        self.radius = radius

    def project(self, axis):
        center_proj = self.pose.x * axis.x + self.pose.y * axis.y
        return center_proj + self.radius, center_proj - self.radius

    def is_inside(self, point):
        distance_squared = (point.x - self.pose.x) ** 2 + (point.y - self.pose.y) ** 2
        return distance_squared <= self.radius ** 2

    def get_center(self):
        return Point(self.pose.x, self.pose.y)
