from abc import abstractmethod

# Geometry
from model.geometry.point import Point
from model.geometry.circle import Circle
from model.geometry.polygon import Polygon
from model.geometry.rectangle import Rectangle
from model.geometry.intersection import check_intersection

from model.world.map.obstacle import Obstacle

# Serialization
import json
import pickle

import numpy as np


class Map:

    def __init__(self,

                 # Obstacles parameters
                 obs_min_width,
                 obs_max_width,
                 obs_min_height,
                 obs_max_height,
                 obs_min_dist,
                 obs_max_dist,
                 obs_count,

                 # Goal parameters
                 goal_min_dist,
                 goal_max_dist,
                 goal_min_clearance,

                 # Global parameters
                 map_boundaries,

                 grid,
                 ):

        # Size of the obstacles (for now, only rectangular obstacles are generated)
        self.obs_min_width = obs_min_width
        self.obs_max_width = obs_max_width
        self.obs_min_height = obs_min_height
        self.obs_max_height = obs_max_height

        # Min and max distance of each obstacle from the center
        self.obs_min_dist = obs_min_dist
        self.obs_max_dist = obs_max_dist

        # Initial number of obstacles
        self.obs_count = obs_count

        # Min and max goal distance from the center
        self.goal_min_dist = goal_min_dist
        self.goal_max_dist = goal_max_dist

        # Clearance between the goal and the obstacles.
        # We will only ever have one goal
        self.goal_min_clearance = goal_min_clearance

        # Obstacles parameters range
        self.obs_width_range = self.obs_max_width - self.obs_min_width
        self.obs_height_range = self.obs_max_height - self.obs_min_height
        self.obs_dist_range = self.obs_max_dist - self.obs_min_dist

        # Map boundaries. If left None, the map will be virtually infinite
        # and have no bounds.
        self.map_boundaries = map_boundaries

        # Whether the map should have a grid structure or not
        self.grid = grid

        # Initial obstacles
        self._initial_obstacles = []

        # Current obstacles
        self._obstacles = []

        # Goal
        self._current_goal = None

        # Enable changes: if True, the map will update the obstacles.
        # Two possible update methods are provided: obstacles can move
        # using their velocity vector or can be randomly spawned
        self.enable_changes = True

    @property
    def goal(self):
        return self._current_goal

    @property
    def obstacles(self):
        return self._obstacles

    @abstractmethod
    def add_obstacle(self, obstacle):
        pass

    @abstractmethod
    def _can_add(self, new_obstacle):
        pass

    def add_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    @abstractmethod
    def spawn_obstacle_at(self, point):
        pass

    def enable(self):
        self.enable_changes = True

    def disable(self):
        self.enable_changes = False

    def query_bounds(self, bounds):
        """
        Query the region defined by (minx, miny, maxx, maxy)
        searching for all the polygons that intersect it
        """

        min_x, min_y, max_x, max_y = bounds
        return self.query_polygon(Polygon([
            Point(min_x, min_y),
            Point(min_x, max_y),
            Point(max_x, max_y),
            Point(max_x, min_y)
        ]))

    @abstractmethod
    def query_polygon(self, polygon):
        """
        Query the region define by the polygon searching for obstacles that intersect it
        """

        pass

    @abstractmethod
    def step_motion(self, dt):
        pass

    @abstractmethod
    def reset(self):
        """
        Reset the map by recovering the initial state of the obstacles.
        We may want to reset other data structures too, that is why
        the method is abstract
        """
        pass

    @abstractmethod
    def clear(self):
        """
        Clear the map by removing everything
        """
        pass

    def to_dict(self):
        return {
            "obstacles": [obstacle.to_dict() for obstacle in self._initial_obstacles],
            "goal": self._current_goal.to_dict()
        }

    # ------------------------------------ IO ------------------------------------ #

    def load_map(self, filename):
        self.load_map_from_json_file(filename)

    def save_map(self, filename):
        self.save_as_json(filename)

    def save_as_pickle(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self, file)

    def save_as_json(self, filename):
        data = self.to_dict()

        with open(filename, "w") as file:
            json.dump(data, file)

    def load_map_from_pickle(self, filename):
        with open(filename, 'rb') as file:
            obj = pickle.load(file)
            self._initial_obstacles = obj._initial_obstacles
            self._obstacles = [obstacle.copy() for obstacle in self._initial_obstacles]
            self._current_goal = obj._current_goal

    def load_map_from_json_file(self, filename):
        with open(filename, 'rb') as file:
            data = json.load(file)
            self.load_map_from_json_data(data)

    def load_map_from_json_data(self, data):
        self.reset()
        self._current_goal = Point.from_dict(data['goal'])
        obstacle_data = data['obstacles']
        for obstacle_dict in obstacle_data:
            obstacle = Obstacle.from_dict(obstacle_dict)
            self.add_obstacle(obstacle)

    # ------------------------------ Map generation ------------------------------ #

    def _generate_random_polygon(self, at=None):

        # Generate dimensions
        width = self.obs_min_width + (np.random.random() * self.obs_width_range)
        height = self.obs_min_height + (np.random.random() * self.obs_height_range)

        # Generate position
        if at is None:
            dist = self.obs_min_dist + (np.random.random() * self.obs_dist_range)
            phi = -np.pi + (np.random.random() * 2 * np.pi)
            x = dist * np.sin(phi)
            y = dist * np.cos(phi)
        else:
            x = at.x
            y = at.y

        theta = np.random.random() * 2 * np.pi - np.pi

        # We have a pose (x, y, theta)

        # If the map should have a grid structure (grid=True)
        # round everything
        if self.grid:
            width = round(width, 1)
            height = round(height, 1)
            x = round(x, 1)
            y = round(y, 1)
            theta = np.random.randint(0, 3) * (np.pi / 2)

        # Crate a polygon
        polygon = Rectangle(width, height)
        polygon.transform(x, y, theta)

        return polygon

    def generate(self, forbidden_zones):

        # clear the map
        self._obstacles = []
        self._initial_obstacles = []

        # Generate the goal
        goal_dist_range = self.goal_max_dist - self.goal_min_dist
        dist = self.goal_min_dist + (np.random.random() * goal_dist_range)
        phi = -np.pi + (np.random.random() * 2 * np.pi)
        x = int(dist * np.sin(phi))  # Round x to an integer
        y = int(dist * np.cos(phi))  # Round y to an integer
        goal = Point(x, y)

        # Generate a proximity test geometry for the goal
        r = self.goal_min_clearance
        goal_test_geometry = Circle(x, y, r)

        # All forbidden zones
        test_geometries = forbidden_zones + [goal_test_geometry]

        # Generate obstacles
        obstacles = []
        while len(obstacles) < self.obs_count:

            polygon = self._generate_random_polygon()

            # Check if the polygon intersects one of the test geometries
            intersects = False
            for test_geometry in test_geometries:
                intersects |= check_intersection(polygon, test_geometry)
                if intersects:
                    break

            # The polygon is good: add the velocity vector and create an obstacle
            if not intersects:
                obstacles.append(Obstacle(polygon))

        # Update the obstacles and the goal
        self._obstacles = obstacles
        self._initial_obstacles = [obstacle.copy() for obstacle in self._obstacles]
        self._current_goal = goal


