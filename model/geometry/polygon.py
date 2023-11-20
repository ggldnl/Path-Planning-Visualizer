from model.geometry.point import Point
from model.geometry.segment import Segment
from model.geometry.shape import Shape

import numpy as np


class Polygon(Shape):

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
            elif isinstance(point, list):
                self.points.append(Point(point[0], point[1]))
            else:
                raise ValueError(f'Invalid object {point}')

        # Super will instantiate the pose object
        super().__init__()

        # Find the center and set pose x and y values
        self._find_center()

        # Find the enclosing radius
        self.radius = self._find_radius()

    @classmethod
    def generate_random_polygon(cls, num_sides, radius, noise=0.5):

        if num_sides < 3:
            raise ValueError('Number of sides must be at least 3')

        angles = np.linspace(0, 2 * np.pi, num_sides, endpoint=False)

        # perturbed_points = []
        perturbed_points = {}

        for angle in angles:

            # Perturb the angle with Gaussian noise
            angle += np.random.normal(0, noise)
            angle = angle % (2 * np.pi)

            # Calculate the coordinates for the random point
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)

            # perturbed_points.append((x, y, angle))
            perturbed_points.update({angle: Point(x, y)})

        # Sort the array to have a convex polygon
        # sorted_perturbed_points = sorted(perturbed_points, key=lambda point: point[2])
        points = [perturbed_points[key] for key in sorted(perturbed_points.keys())]

        return cls(points)

    @classmethod
    def from_dict(cls, dictionary):

        # point_list = json.loads(dictionary['points'], object_hook=lambda d: Point(d['x'], d['y']))

        points = []
        for point_dictionary in dictionary['points']:
            points.append(Point.from_dict(point_dictionary))

        return Polygon(points)

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

        return min_x, min_y, max_x, max_y

    def _find_radius(self):
        """
        Supposes that the center has already been found
        """

        radius = 0
        for point in self.points:
            distance = np.sqrt((self.pose.x - point.x) ** 2 + (self.pose.y - point.y) ** 2)
            radius = max(radius, distance)

        return radius

    def _find_center(self):
        total_x = sum(point.x for point in self.points)
        total_y = sum(point.y for point in self.points)
        num_points = len(self.points)
        self.pose.x = total_x / num_points
        self.pose.y = total_y / num_points

    def translate(self, offset_x, offset_y):

        for point in self.points:
            point.x += offset_x
            point.y += offset_y

        self.pose.x += offset_x
        self.pose.y += offset_y

    def rotate_around_pose(self, pose):
        x, y, theta = pose
        self.rotate_around(x, y, theta)

    def rotate_around(self, x, y, angle):
        """
        Rotate the polygon around a specified point by the specified angle (in radians).
        """

        if not 0 <= angle <= 2 * np.pi:
            raise ValueError(f'Angle {angle} is not in radians')

        # Apply rotation to each point
        for point in self.points:

            # Translate the point to the origin (center) of rotation
            translated_x = point.x - x
            translated_y = point.y - y

            # Perform the rotation
            new_x = translated_x * np.cos(angle) - translated_y * np.sin(angle)
            new_y = translated_x * np.sin(angle) + translated_y * np.cos(angle)

            # Translate the point back to its original position
            point.x = new_x + x
            point.y = new_y + y

        # Update the center
        self._find_center()

    def rotate(self, angle):
        """
        Rotate around the center by the specified angle
        """

        # Apply rotation to each point
        for point in self.points:
            # Translate the point to the origin (center) of rotation
            translated_x = point.x - self.pose.x
            translated_y = point.y - self.pose.y

            # Perform the rotation
            new_x = translated_x * np.cos(angle) - translated_y * np.sin(angle)
            new_y = translated_x * np.sin(angle) + translated_y * np.cos(angle)

            # Translate the point back to its original position
            point.x = new_x + self.pose.x
            point.y = new_y + self.pose.y

    def transform(self, pose):
        x, y, alpha = pose
        self.translate(x, y)
        self.rotate(alpha)

    def translate_to(self, x, y):
        offset_x = x - self.pose.x
        offset_y = y - self.pose.y
        self.translate(offset_x, offset_y)

    def rotate_to(self, target_angle):
        # Compute the angle difference
        angle_diff = target_angle - self.pose.theta
        self.rotate(angle_diff)

    def transform_to_pose(self, pose):
        x, y, theta = pose
        self.translate_to(x, y)
        self.rotate_to(theta)

    def transform_to(self, x, y, theta):
        self.translate_to(x, y)
        self.rotate_to(theta)

    def get_edges(self):
        # Get the edges of the polygon
        edges = []
        for i in range(len(self.points)):
            edge = Segment(self.points[i], self.points[(i + 1) % len(self.points)])
            edges.append(edge)
        return edges

    def check_nearness(self, other):
        return self.pose.distance(other.pose) <= self.radius + other.radius

    def project(self, axis):
        """
        Project the polygon onto an axis and return the min and max values
        """
        min_proj = float('inf')
        max_proj = float('-inf')
        for point in self.points:
            projection = point.x * axis.x + point.y * axis.y
            if projection < min_proj:
                min_proj = projection
            if projection > max_proj:
                max_proj = projection
        return min_proj, max_proj

    def copy(self):
        """
        Returns a deep copy of the polygon
        """
        # points = []
        # for point in self.points:
        #     points.append(Point(point.x, point.y))
        # return Polygon(points)

        return Polygon([point.copy() for point in self.points])

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

    def __getitem__(self, item):

        if item < 0 or item > len(self.points) - 1:
            raise IndexError(f'Polygon point index out of range: {item}')
        return self.points[item]

    def __setitem__(self, key, value):

        if not isinstance(value, Point):
            raise ValueError(f'Invalid object {type(value)}; must be Point.')

        if key < 0 or key > len(self.points) - 1:
            raise IndexError(f'Polygon point index out of range: {key}')

        self.points[key] = value
        self.radius = self._find_radius()
        self._find_center()
