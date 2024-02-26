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

    def generate_point_inside(self):
        # Generate a random angle
        theta = np.random.uniform(0, 2 * np.pi)
        # Generate a random radius
        r = np.sqrt(np.random.uniform(0, 1))

        # Scale by the semi-axes
        x_prime = self.a * r * np.cos(theta)
        y_prime = self.b * r * np.sin(theta)

        # Rotate by phi
        cos_phi = np.cos(self.phi)
        sin_phi = np.sin(self.phi)
        x_rotated = x_prime * cos_phi - y_prime * sin_phi
        y_rotated = x_prime * sin_phi + y_prime * cos_phi

        # Translate to the center
        x = self.pose.x + x_rotated
        y = self.pose.y + y_rotated

        return {'x': x, 'y': y}

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

    @staticmethod
    def from_path_points(a1, b1, a2, b2, c):
        # Compute ellipse parameters
        a = c / 2  # Semimajor axis
        x0 = (a1 + a2) / 2  # Center x-value
        y0 = (b1 + b2) / 2  # Center y-value
        f = np.sqrt((a1 - x0) ** 2 + (b1 - y0) ** 2)  # Distance from center to focus

        # Check if the expression inside np.sqrt is non-negative
        if a ** 2 - f ** 2 < 0:
            # If negative, set b to a small positive value
            b = 0.01
        else:
            b = np.sqrt(a ** 2 - f ** 2)  # Semiminor axis

        phi = np.arctan2((b2 - b1), (a2 - a1))  # Angle betw major axis and x-axis

        return Ellipse(x0, y0, a, b, phi)

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
