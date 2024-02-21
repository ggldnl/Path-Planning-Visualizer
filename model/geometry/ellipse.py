from model.geometry.shape import Shape
from model.geometry.point import Point
import numpy as np


class Ellipse(Shape):

    def __init__(self, x0, y0, a, b, phi):
        super().__init__()

        self.pose.x = x0
        self.pose.y = y0
        self.a = a  # Semimajor axis
        self.b = b  # Semiminor axis
        self.phi = phi  # Rotation angle of the major axis

        self.points = [self.calculate_point(theta) for theta in np.linspace(0, 2 * np.pi, 24)]

    def calculate_point(self, theta):
        x = self.a * np.cos(theta)
        y = self.b * np.sin(theta)
        x_rot = x * np.cos(self.phi) - y * np.sin(self.phi)
        y_rot = x * np.sin(self.phi) + y * np.cos(self.phi)
        return Point(self.pose.x + x_rot, self.pose.y + y_rot)

    def get_bounds(self):
        # Approximate bounds by calculating points and finding min/max
        xs = [point.x for point in self.points]
        ys = [point.y for point in self.points]
        return (min(xs), min(ys), max(xs), max(ys))

    def is_inside(self, point):
        # Check if a point is inside the ellipse using the ellipse equation
        # Rotate and translate the point to align with the ellipse axes
        cos_phi = np.cos(-self.phi)
        sin_phi = np.sin(-self.phi)
        x_rot = cos_phi * (point.x - self.pose.x) - sin_phi * (point.y - self.pose.y)
        y_rot = sin_phi * (point.x - self.pose.x) + cos_phi * (point.y - self.pose.y)
        return (x_rot / self.a) ** 2 + (y_rot / self.b) ** 2 <= 1

    def rotate(self, theta):
        self.phi += theta
        self.points = [self.calculate_point(theta) for theta in np.linspace(0, 2 * np.pi, 24)]

    def to_dict(self):
        return {
            'x': self.pose.x,
            'y': self.pose.y,
            'a': self.a,
            'b': self.b,
            'phi': self.phi
        }

    def __str__(self):
        return f'Ellipse(center={self.pose.as_point()}, a={self.a}, b={self.b}, phi={self.phi})'

    def __repr__(self):
        return self.__str__()

    def copy(self):
        return Ellipse(self.pose.x, self.pose.y, self.a, self.b, self.phi)
