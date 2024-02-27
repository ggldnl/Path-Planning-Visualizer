from model.geometry.shape import Shape
from model.geometry.point import Point
import numpy as np


class Ellipse(Shape):

    def __init__(self, center, a, b, phi):
        super().__init__()

        self.pose.x = center.x
        self.pose.y = center.y
        self.pose.theta = phi
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

        # Calculate the bounds without generating all points
        cos_theta = np.cos(np.radians(self.phi))
        sin_theta = np.sin(np.radians(self.phi))

        # Calculate the extreme points on the ellipse
        x1 = self.pose.x + self.a * cos_theta
        x2 = self.pose.x - self.a * cos_theta
        y1 = self.pose.y + self.a * sin_theta
        y2 = self.pose.y - self.a * sin_theta

        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)

        return min_x, min_y, max_x, max_y

    def is_inside(self, point):
        # Check if a point is inside the ellipse using the ellipse equation
        # Rotate and translate the point to align with the ellipse axes
        cos_phi = np.cos(-self.phi)
        sin_phi = np.sin(-self.phi)
        x_rot = cos_phi * (point.x - self.pose.x) - sin_phi * (point.y - self.pose.y)
        y_rot = sin_phi * (point.x - self.pose.x) + cos_phi * (point.y - self.pose.y)
        return (x_rot / self.a) ** 2 + (y_rot / self.b) ** 2 <= 1

    @staticmethod
    def from_path_points(focus1, focus2, major_axis_length):

        # Calculate the distance between the two foci
        distance_foci = focus1.distance(focus2)

        # Calculate the semi-major axis
        semi_major_axis = major_axis_length / 2

        # Calculate the semi-minor axis using the ellipse equation: b = sqrt(a^2 - c^2)
        semi_minor_axis = np.sqrt((semi_major_axis ** 2) - (distance_foci / 2) ** 2)

        # Calculate the center of the ellipse
        center = Point((focus1[0] + focus2[0]) / 2, (focus1[1] + focus2[1]) / 2)

        # Calculate the angle of rotation (if any) of the ellipse
        angle_rad = np.arctan2(focus2[1] - focus1[1], focus2[0] - focus1[0])

        # Return the parameters of the ellipse
        return Ellipse(center, semi_major_axis, semi_minor_axis, -angle_rad)

    """
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
    """

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
        return Ellipse(self.pose.as_point(), self.a, self.b, self.phi)
