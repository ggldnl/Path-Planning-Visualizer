from model.geometry.shape import Shape
from model.geometry.point import Point
import numpy as np


class Circle(Shape):

    def __init__(self, x, y, radius):
        super().__init__()

        self.pose.x = x
        self.pose.y = y
        self.radius = radius

        # Set of points to discretize the shape, needed to compute convex hull
        self.points = [Point(radius * np.cos(angle), radius * np.sin(angle))
                       for angle in np.linspace(0, 2 * np.pi, 12)]

    def get_bounds(self):
        return (self.pose.x - self.radius,
                self.pose.y - self.radius,
                self.pose.x + self.radius,
                self.pose.y + self.radius)

    @classmethod
    def segment_buffer(cls, segment, margin=None):
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

    def translate(self, offset_x, offset_y):
        self.pose.x += offset_x
        self.pose.y += offset_y

        for point in self.points:
            point.x += offset_x
            point.y += offset_y

    def rotate(self, theta):
        pass

    def transform(self, x, y, theta):
        self.translate(x, y)

    def translate_to(self, x, y):
        self.pose.x = x
        self.pose.y = y

        for point in self.points:
            point.x = x
            point.y = y

    def rotate_to(self, theta):
        pass

    def transform_to(self, x, y, theta):
        self.translate_to(x, y)

    def rotate_around(self, x, y, theta):

        # Compute the center coordinates
        translated_x = self.pose.x - x
        translated_y = self.pose.y - y

        # Rotate the coordinates
        self.pose.x = x + translated_x * np.cos(theta) - translated_y * np.sin(theta)
        self.pose.y = y + translated_x * np.sin(theta) + translated_y * np.cos(theta)

        # Update the discretization points
        for point in self.points:

            translated_x = point.x - x
            translated_y = point.y - y

            point.x = x + translated_x * np.cos(theta) - translated_y * np.sin(theta)
            point.y = y + translated_x * np.sin(theta) + translated_y * np.cos(theta)

    def to_dict(self):
        return {
            'x': self.pose.x,
            'y': self.pose.y,
            'radius': self.radius
        }

    def get_edges(self):
        pass

    def copy(self):
        return Circle(self.pose.x, self.pose.y, self.radius)

    def __str__(self):
        return f'Circle(center={self.pose.as_point()}, radius={self.radius})'

    def __repr__(self):
        return self.__str__()
