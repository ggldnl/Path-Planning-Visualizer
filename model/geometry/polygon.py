from model.geometry.point import Point
import model.geometry.linalg as linalg

import numpy as np
import math


class Polygon:

    def __init__(self, points):
        """
        :param points: a list of 2-dimensional vectors.
        """

        # Deep copy of the points array. While making the copy we can
        # find the centroid of the polygon

        self.points = []
        for point in points:
            if isinstance(point, Point):
                self.points.append(Point(point.x, point.y))  # copy it
            elif isinstance(point, tuple):
                self.points.append(Point(point[0], point[1]))
            else:
                raise ValueError(f'Invalid object {point}')

        # Compute centroid and radius of the circumference
        self._compute_centroid()
        self._compute_radius()

    @classmethod
    def generate_random_polygon(cls, num_sides, radius, noise=0.5):

        if num_sides < 3:
            raise ValueError("Number of sides must be at least 3")

        angles = np.linspace(0, 2 * np.pi, num_sides, endpoint=False)

        # perturbed_points = []
        perturbed_points = {}

        for angle in angles:

            # Perturb the angle with Gaussian noise
            angle += np.random.normal(0, noise)

            # Calculate the coordinates for the random point
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)

            # perturbed_points.append((x, y, angle))
            perturbed_points.update({angle: Point(x, y)})

        # Sort the array to have a convex polygon
        # sorted_perturbed_points = sorted(perturbed_points, key=lambda point: point[2])
        points = [perturbed_points[key] for key in sorted(perturbed_points.keys())]

        return cls(points)

    def add(self, point):
        """
        Add a point to the polygon
        """

        self.points.append(point)
        self._compute_centroid()
        self._compute_radius()

    def to_point_array(self):
        return [[point.x, point.y] for point in self.points]

    def to_dict(self):
        return {'points': [point.to_dict() for point in self.points]}

    def get_bounding_box(self):
        """
        Returns the bounding box of the polygon.
        """

        # Compute the bounding box (list of points)
        min_x = self.points[0].x
        max_x = self.points[0].x
        min_y = self.points[0].y
        max_y = self.points[0].y

        for point in self.points:
            min_x = min(min_x, point.x)
            max_x = max(max_x, point.x)
            min_y = min(min_y, point.y)
            max_y = max(max_y, point.y)

        return Polygon([
            Point(min_x, min_y),
            Point(min_x, max_y),
            Point(max_x, max_y),
            Point(max_x, min_y)
        ])

    def get_bounding_circle(self):
        """
        Get the center point and radius for a circle that completely contains this polygon.

        NOTE: this method is meant to give a quick bounding circle
                the circle calculated may not be the minimum bounding circle.
        """

        return self.centroid, self.bounding_circle_radius

    def scale(self, scale_x, scale_y):
        """
        Scale the polygon by a factor on the bounding box width and height
        preserving the bounding box center (self.centroid).
        """

        # Translate to the origin
        for point in self.points:
            point -= self.centroid

        # Scale the points
        for point in self.points:
            point.x *= scale_x
            point.y *= scale_y

        # Translate back to original position
        for point in self.points:
            point += self.centroid

        # Update bounding circle size, centroid stays the same
        self._compute_radius()

    def translate(self, offset_x, offset_y):
        """
        Translates all the points of the polygon by an offset in x and y direction.
        """

        for point in self.points:
            point.x = point.x + offset_x
            point.y = point.y + offset_y

        # update center and bounding box size
        self._compute_centroid()

    def translate_to(self, x, y):
        """
        Translates all the points in the polygon such that the center of
        the bounding box falls on (x, y).
        """

        offset_x = x - self.centroid.x
        offset_y = y - self.centroid.y

        self.translate(offset_x, offset_y)

    def get_transformation(self, pose):
        """
        Return a copy of this polygon transformed to the given pose.
        The pose is a (x, y, theta) tuple
        """
        x, y, theta = pose
        return Polygon(
            linalg.rotate_and_translate_vectors(self.points, Point(x, y), theta)
        )

    def rotate(self, theta):
        """
        Rotate the polygon by the angle theta (radians)
        """

        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)

        for point in self.points:
            point.x = point.x * cos_theta - point.y * sin_theta
            point.y = point.x * sin_theta + point.y * cos_theta

    def transform(self, pose):
        """
        Rotate and translate the polygon. Pose is a 3 element tuple (x, y, theta)
        """
        self.translate(pose[0], pose[1])
        self.rotate(pose[2])

    def transform_to(self, pose):
        """
        Rotate and translate the polygon. Pose is a 3 element tuple (x, y, theta)
        """
        self.translate_to(pose[0], pose[1])
        self.rotate(pose[2])

    def get_edges(self):
        """
        Get a list of this polygon's edges as Point pairs
        """

        n = len(self.points)
        edges = []

        for i in range(n):
            edges.append([self.points[i], self.points[(i + 1) % n]])

        return edges

    def _compute_centroid(self):
        """
        Approximate the centroid of this polygon.

        NOTE: this method is meant to give a quick and dirty approximation of center
                of the polygon. It returns the average of the vertices; the actual
                centroid may not be equivalent.
        """

        # Compute the centroid
        self.centroid = Point(0, 0)
        for point in self.points:
            self.centroid += point  # add it to the centroid
        self.centroid /= len(self.points)

    def _compute_radius(self):
        """
        Compute the radius of a circle that completely contains the polygon
        """

        # Find the radius of the bounding circle
        self.bounding_circle_radius = self.centroid.distance(self.points[0])
        for i in range(1, len(self.points)):
            distance = self.centroid.distance(self.points[i])
            self.bounding_circle_radius = max(self.bounding_circle_radius, distance)

    def copy(self):
        """
        Returns a deep copy of the polygon
        """
        points = []
        for point in self.points:
            points.append(Point(point.x, point.y))
        return Polygon(points)

    def __eq__(self, other):

        if isinstance(other, Polygon):

            if len(self.points) != len(other.points):
                return False

            # Check equality for each point
            for p1, p2 in zip(self.points, other.points):
                if p1 != p2:
                    return False
            return True

        return False

    def __len__(self):
        """
        Return the number of vertex of the polygon
        """

        return len(self.points)

    def __str__(self):
        point_str = ', '.join(str(point) for point in self.points)
        return f"Polygon(points=[{point_str}])"
