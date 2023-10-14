# Math
from math import pi, sin, cos
import random

# Geometry
from model.geometry.polygon import Polygon
from model.geometry.point import Point
import model.geometry.utils as utils

# Obstacles
from model.world.map.rectangular_obstacle import RectangularObstacle

# Serialization
import pickle
import json


# random environment parameters
OBS_MIN_DIM = 0.1  # meters
OBS_MAX_DIM = 2.5  # meters
OBS_MIN_COUNT = 10
OBS_MAX_COUNT = 50
OBS_MIN_DIST = 0.4  # meters
OBS_MAX_DIST = 6.0  # meters
GOAL_MIN_DIST = 2.0  # meters
GOAL_MAX_DIST = 4.0  # meters
MIN_GOAL_CLEARANCE = 0.2  # meters


class Map:

    def __init__(self):

        # Initial obstacle position
        self.obstacles = []

        # Current obstacle position
        self.current_obstacles = []

        self.current_goal = None

    def random_map(self, robots):

        # Obstacle parameters
        obs_min_dim = OBS_MIN_DIM
        obs_max_dim = OBS_MAX_DIM
        obs_min_count = OBS_MIN_COUNT
        obs_max_count = OBS_MAX_COUNT
        obs_min_dist = OBS_MIN_DIST
        obs_max_dist = OBS_MAX_DIST

        # Goal parameters
        goal_min_dist = GOAL_MIN_DIST
        goal_max_dist = GOAL_MAX_DIST

        # Generate the goal
        goal_dist_range = goal_max_dist - goal_min_dist
        dist = goal_min_dist + (random.random() * goal_dist_range)
        phi = -pi + (random.random() * 2 * pi)
        x = dist * sin(phi)
        y = dist * cos(phi)
        goal = Point(x, y)

        # Generate a proximity test geometry for the goal
        r = MIN_GOAL_CLEARANCE
        n = 6
        goal_test_geometry = []
        for i in range(n):
            goal_test_geometry.append(
                Point(x + r * cos(i * 2 * pi / n), y + r * sin(i * 2 * pi / n))
            )
        goal_test_geometry = Polygon(goal_test_geometry)

        # Generate the obstacles
        obstacles = []
        obs_dim_range = obs_max_dim - obs_min_dim
        obs_dist_range = obs_max_dist - obs_min_dist
        num_obstacles = random.randrange(obs_min_count, obs_max_count + 1)

        # test_geometries contains the robots and the goal
        test_geometries = [r.body for r in robots] + [
            goal_test_geometry
        ]

        while len(obstacles) < num_obstacles:

            # Generate dimensions
            width = obs_min_dim + (random.random() * obs_dim_range)
            height = obs_min_dim + (random.random() * obs_dim_range)

            # Generate position
            dist = obs_min_dist + (random.random() * obs_dist_range)
            phi = -pi + (random.random() * 2 * pi)
            x = dist * sin(phi)
            y = dist * cos(phi)

            # Generate orientation
            theta = -pi + (random.random() * 2 * pi)

            # Generate velocity
            vel = (0, 0, 0)

            # Test if the obstacle overlaps the robots or the goal.
            # We need to set the velocity vector as (0, 0, 0) to make the object
            # static for the check
            obstacle = RectangularObstacle(width, height, (x, y, theta), vel)
            intersects = False
            for test_geometry in test_geometries:
                intersects |= utils.convex_polygon_intersect_test(
                    test_geometry, obstacle.polygon
                )
            if not intersects:
                obstacles.append(obstacle)

        # Update the velocity vector
        for obstacle in obstacles:
            obstacle.vel = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

        # Update the obstacles and the goal
        self.current_obstacles = obstacles
        self.obstacles = obstacles
        self.current_goal = goal

    def reset_map(self):
        self.current_obstacles = self.obstacles

    def save_map(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self.current_obstacles, file)
            pickle.dump(self.current_goal, file)

    def load_map(self, filename):
        with open(filename, "rb") as file:
            self.current_obstacles = pickle.load(file)
            self.current_goal = pickle.load(file)
