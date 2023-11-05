from model.geometry.point import Point
from model.geometry.segment import Segment

import numpy as np
import json


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
            elif isinstance(point, list):
                self.points.append(Point(point[0], point[1]))
            else:
                raise ValueError(f'Invalid object {point}')

        # Initialize internal angle to 0 degrees
        self.angle = 0

        self.center = self._find_center()
        self.radius = self._find_radius()

    @property
    def bounds(self):
        return self.get_bounding_box(as_tuple=True)

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

    def to_point_array(self):
        return [[point.x, point.y] for point in self.points]

    def to_dict(self):
        return {'points': [point.to_dict() for point in self.points]}

    @classmethod
    def from_dict(cls, dictionary):

        # point_list = json.loads(dictionary['points'], object_hook=lambda d: Point(d['x'], d['y']))

        points = []
        for point_dictionary in dictionary['points']:
            points.append(Point.from_dict(point_dictionary))

        return Polygon(points)

    def get_bounding_box(self, as_tuple=False):
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

        if as_tuple:
            return (min_x, min_y, max_x, max_y)

        return Polygon([
            Point(min_x, min_y),
            Point(min_x, max_y),
            Point(max_x, max_y),
            Point(max_x, min_y)
        ])

    def _find_radius(self):
        """
        Supposes that the center has already been found
        """

        radius = 0
        for point in self.points:
            distance = self.center.distance(point)
            radius = max(radius, distance)

        return radius

    def translate(self, offset_x, offset_y):

        for point in self.points:
            point.x += offset_x
            point.y += offset_y

        # Update the center
        self.center = self._find_center()

    def rotate_around(self, x, y, angle, is_deg=True):
        """
        Rotate the polygon around a specified point by the specified angle.
        """

        if is_deg:
            angle_radians = np.deg2rad(angle)
        else:
            angle_radians = angle

        # Apply rotation to each point
        for point in self.points:
            # Translate the point to the origin (center) of rotation
            translated_x = point.x - x
            translated_y = point.y - y

            # Perform the rotation
            new_x = translated_x * np.cos(angle_radians) - translated_y * np.sin(angle_radians)
            new_y = translated_x * np.sin(angle_radians) + translated_y * np.cos(angle_radians)

            # Translate the point back to its original position
            point.x = new_x + x
            point.y = new_y + y

        self.center = self._find_center()

    def rotate(self, angle, is_deg=True):
        """
        Rotate around the center by the specified angle
        """

        if is_deg:
            angle_radians = np.deg2rad(angle)
            self.angle += angle
        else:
            angle_radians = angle
            self.angle += np.rad2deg(angle)

        # Apply rotation to each point
        for point in self.points:
            # Translate the point to the origin (center) of rotation
            translated_x = point.x - self.center.x
            translated_y = point.y - self.center.y

            # Perform the rotation
            new_x = translated_x * np.cos(angle_radians) - translated_y * np.sin(angle_radians)
            new_y = translated_x * np.sin(angle_radians) + translated_y * np.cos(angle_radians)

            # Translate the point back to its original position
            point.x = new_x + self.center.x
            point.y = new_y + self.center.y

    def transform(self, pose, is_deg=True):
        x, y, alpha = pose
        self.translate(x, y)
        self.rotate(alpha, is_deg)

    def translate_to(self, x, y):
        offset_x = x - self.center.x
        offset_y = y - self.center.y
        self.translate(offset_x, offset_y)

    def rotate_to(self, target_angle, is_deg=True):
        # Compute the angle difference
        angle_diff = target_angle - self.angle
        self.rotate(angle_diff, is_deg)

    def transform_to(self, pose, is_deg=True):
        self.translate_to(pose[0], pose[1])
        #self.rotate_to(pose[2], is_deg)

    def _find_center(self):
        total_x = sum(point.x for point in self.points)
        total_y = sum(point.y for point in self.points)
        num_points = len(self.points)
        center_x = total_x / num_points
        center_y = total_y / num_points
        return Point(center_x, center_y)

    def get_edges(self):
        # Get the edges of the polygon
        edges = []
        for i in range(len(self.points)):
            edge = (self.points[i], self.points[(i + 1) % len(self.points)])
            edges.append(edge)
        return edges

    @classmethod
    def _normal(cls, edge):
        # Calculate the normal vector of an edge
        p1, p2 = edge
        return Point(-(p2.y - p1.y), p2.x - p1.x)

    def _project(self, axis):
        # Project the polygon onto an axis and return the min and max values
        min_proj = float('inf')
        max_proj = float('-inf')
        for point in self.points:
            projection = point.x * axis.x + point.y * axis.y
            if projection < min_proj:
                min_proj = projection
            if projection > max_proj:
                max_proj = projection
        return min_proj, max_proj

    def intersects(self, other):

        # Check if the input is a Polygon or a Line
        if isinstance(other, Polygon):
            edges_other = other.get_edges()
        elif isinstance(other, Segment):
            edges_other = [(other.start, other.end)]
        else:
            raise ValueError("Input must be a Polygon or a Line")

        for edge in self.get_edges() + edges_other:
            axis = self._normal(edge)
            min1, max1 = self._project(axis)
            min2, max2 = other._project(axis)

            if max1 < min2 or max2 < min1:
                # If there is a gap along this axis, the polygons do not intersect
                return False

        return True

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
