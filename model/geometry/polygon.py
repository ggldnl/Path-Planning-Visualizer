from model.geometry.point import Point

import numpy as np


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

    def add(self, point):
        self.points.append(point)

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

    def translate(self, offset_x, offset_y):
        for point in self.points:
            point.x += offset_x
            point.y += offset_y

    def rotate(self, angle_degrees):

        center_x, center_y = self.find_center()
        angle_radians = np.deg2rad(angle_degrees)
        self.angle += angle_degrees

        # Apply rotation to each point
        for point in self.points:
            # Translate the point to the origin (center) of rotation
            translated_x = point.x - center_x
            translated_y = point.y - center_y

            # Perform the rotation
            new_x = translated_x * np.cos(angle_radians) - translated_y * np.sin(angle_radians)
            new_y = translated_x * np.sin(angle_radians) + translated_y * np.cos(angle_radians)

            # Translate the point back to its original position
            point.x = new_x + center_x
            point.y = new_y + center_y

    def transform(self, pose):
        x, y, alpha = pose
        self.translate(x, y)
        self.rotate(alpha)

    def translate_to(self, x, y):
        center_x, center_y = self.find_center()
        offset_x = x - center_x
        offset_y = y - center_y
        self.translate(offset_x, offset_y)

    def rotate_to(self, target_angle):
        # Compute the angle difference
        angle_diff = target_angle - self.angle
        self.rotate(angle_diff)

    def transform_to(self, pose):
        self.translate_to(pose[0], pose[1])
        self.rotate_to(pose[2])

    def find_center(self):
        total_x = sum(point.x for point in self.points)
        total_y = sum(point.y for point in self.points)
        num_points = len(self.points)
        center_x = total_x / num_points
        center_y = total_y / num_points
        return center_x, center_y

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

    def intersects(self, other_polygon):
        for edge in self.get_edges() + other_polygon.get_edges():
            axis = self._normal(edge)
            min1, max1 = self._project(axis)
            min2, max2 = other_polygon._project(axis)

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
