from abc import abstractmethod

# Math
import numpy as np

# Model
from model.geometry.rectangle import Rectangle
from model.world.map.obstacle import Obstacle
from model.geometry.point import Point
from model.geometry.circle import Circle
from model.geometry.intersection import check_intersection

# Serialization
import json


class Map:

    def __init__(self,
                 # Obstacle parameters
                 obs_min_dist,
                 obs_max_dist,
                 obs_min_width,
                 obs_max_width,
                 obs_min_height,
                 obs_max_height,
                 obs_count,

                 # Speed
                 obs_min_lin_speed,
                 obs_max_lin_speed,
                 obs_min_ang_speed,
                 obs_max_ang_speed,

                 # Goal parameters
                 goal_min_dist,
                 goal_max_dist,
                 min_goal_clearance,

                 map_type,

                 boundaries
                 ):

        # Set parameters

        # Distance from the center (spawning point)
        self.obs_min_dist = obs_min_dist
        self.obs_max_dist = obs_max_dist

        # Dimension of the obstacles
        self.obs_min_width = obs_min_width
        self.obs_max_width = obs_max_width
        self.obs_min_height = obs_min_height
        self.obs_max_height = obs_max_height

        # Number of obstacles
        self.obs_count = obs_count

        # Obstacle speed
        self.obs_min_lin_speed = obs_min_lin_speed
        self.obs_max_lin_speed = obs_max_lin_speed
        self.obs_min_ang_speed = obs_min_ang_speed
        self.obs_max_ang_speed = obs_max_ang_speed

        # Goal distance from the spawning point
        self.goal_min_dist = goal_min_dist
        self.goal_max_dist = goal_max_dist

        self.min_goal_clearance = min_goal_clearance

        # Obstacles parameters range
        self.obs_width_range = self.obs_max_width - self.obs_min_width
        self.obs_height_range = self.obs_max_height - self.obs_min_height
        self.obs_dist_range = self.obs_max_dist - self.obs_min_dist

        self.map_type = map_type  # spatial | normal

        self.boundaries = boundaries

        # Allow changes in map (if map supports moving obstacles, enable
        # moving obstacles, if map supports randomly generated obstacles,
        # enable spawn)
        self.allow_changes = True

        # Initial obstacles
        self._initial_obstacles = []

        # Current obstacles
        self._obstacles = []

        self._current_goal = None

    @property
    def goal(self):
        return self._current_goal

    @property
    def obstacles(self):
        return self._obstacles

    @abstractmethod
    def add_obstacle(self, obstacle):
        pass

    def enable_changes(self):
        self.allow_changes = True

    def disable_changes(self):
        self.allow_changes = False

    def add_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    def _generate_random_obstacle_polygon(self):

        # TODO implement other obstacle types

        # Generate dimensions
        width = self.obs_min_width + (np.random.random() * self.obs_width_range)
        height = self.obs_min_height + (np.random.random() * self.obs_height_range)

        # Generate position
        dist = self.obs_min_dist + (np.random.random() * self.obs_dist_range)
        phi = -np.pi + (np.random.random() * 2 * np.pi)
        x = dist * np.sin(phi)
        y = dist * np.cos(phi)

        theta = np.random.random() * 2 * np.pi - np.pi

        # We have a pose
        pose = (x, y, theta)

        # Create a polygon
        polygon = Rectangle(width, height)
        polygon.transform(pose)

        return polygon

    def generate(self, robots):

        # Generate the goal
        goal_dist_range = self.goal_max_dist - self.goal_min_dist
        dist = self.goal_min_dist + (np.random.random() * goal_dist_range)
        phi = -np.pi + (np.random.random() * 2 * np.pi)
        x = dist * np.sin(phi)
        y = dist * np.cos(phi)
        goal = Point(x, y)

        # Generate a proximity test geometry for the goal
        r = self.min_goal_clearance

        """
        n = 6
        goal_test_geometry = []
        for i in range(n):
            goal_test_geometry.append(
                Point(x + r * cos(i * 2 * pi / n), y + r * sin(i * 2 * pi / n))
            )
        goal_test_geometry = Polygon(goal_test_geometry)
        """
        goal_test_geometry = Circle(x, y, r)

        # Generate a proximity test geometry for the robots
        robot_dist = self.obs_min_dist
        robots_proximity_polygons_test = [Circle(r.outline.pose.x, r.outline.pose.y, robot_dist) for r in robots]

        test_geometries = robots_proximity_polygons_test + [goal_test_geometry]

        # Generate obstacles
        obstacles = []
        while len(obstacles) < self.obs_count:

            polygon = self._generate_random_obstacle_polygon()

            # Check if the polygon intersects one of the test geometries
            intersects = False
            for test_geometry in test_geometries:
                intersects |= check_intersection(polygon, test_geometry)
                if intersects:
                    break

            # The polygon is good: add the velocity vector and create an obstacle
            if not intersects:
                obstacle = Obstacle(polygon)
                obstacles.append(obstacle)

        # Update the obstacles and the goal
        self.add_obstacles(obstacles)
        self._current_goal = goal

    def get_polygon(self, obj_id):
        # Retrieve the polygon geometry based on its identifier
        return self._obstacles[obj_id].polygon

    @abstractmethod
    def query_region(self, region):
        """
        Returns all the obstacles in the region polygon
        """
        pass

    @abstractmethod
    def query(self, bounds):
        pass

    @abstractmethod
    def step_motion(self, dt):
        # Obstacles behavior
        pass

    @abstractmethod
    def reset_map(self):
        pass

    @abstractmethod
    def clear(self):
        pass

    def load_map(self, filename):
        self.load_map_from_json_file(filename)

    def save_map(self, filename):
        self.save_as_json(filename)

    @abstractmethod
    def save_as_pickle(self, filename):
        pass

    @abstractmethod
    def save_as_json(self, filename):
        pass

    @abstractmethod
    def load_map_from_pickle(self, filename):
        pass

    def load_map_from_json_file(self, filename):
        with open(filename, 'rb') as file:
            data = json.load(file)
            self.load_map_from_json_data(data)

    @abstractmethod
    def load_map_from_json_data(self, data):
        pass
