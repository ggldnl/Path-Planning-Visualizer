# Math
from math import pi, sin, cos
import random

# Model
from model.geometry.rectangle import Rectangle
from model.world.map.obstacle import Obstacle
from model.geometry.polygon import Polygon
from model.geometry.point import Point

# Serialization
import pickle
import json

# random environment parameters
OBS_MIN_WIDTH = 5
OBS_MAX_WIDTH = OBS_MIN_WIDTH + 5
OBS_MIN_HEIGHT = 5
OBS_MAX_HEIGHT = OBS_MIN_HEIGHT + 5
OBS_MIN_COUNT = 10
OBS_MAX_COUNT = 50
OBS_MIN_DIST = 20.0  # meters
OBS_MAX_DIST = 50.0  # meters
GOAL_MIN_DIST = 10.0  # meters
GOAL_MAX_DIST = 50.0  # meters
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
        obs_min_count = OBS_MIN_COUNT
        obs_max_count = OBS_MAX_COUNT
        obs_min_dist = OBS_MIN_DIST
        obs_max_dist = OBS_MAX_DIST
        obs_min_width = OBS_MIN_WIDTH
        obs_max_width = OBS_MAX_WIDTH
        obs_min_height = OBS_MIN_HEIGHT
        obs_max_height = OBS_MAX_HEIGHT

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
        obs_width_range = obs_max_width - obs_min_width
        obs_height_range = obs_max_height - obs_min_height
        obs_dist_range = obs_max_dist - obs_min_dist
        num_obstacles = random.randrange(obs_min_count, obs_max_count + 1)

        # test_geometries contains the robots and the goal
        test_geometries = [r.body for r in robots] + [
            goal_test_geometry
        ]

        while len(obstacles) < num_obstacles:

            # Generate dimensions
            width = obs_min_width + (random.random() * obs_width_range)
            height = obs_min_height + (random.random() * obs_height_range)

            # Generate position
            dist = obs_min_dist + (random.random() * obs_dist_range)
            phi = -pi + (random.random() * 2 * pi)
            x = dist * sin(phi)
            y = dist * cos(phi)

            # Generate orientation
            theta = -pi + (random.random() * 2 * pi)

            # We have a pose
            pose = (x, y, theta)

            # Create a polygon
            polygon = Rectangle(width, height)
            polygon.transform(pose)

            # Check if the polygon intersects one of the test geometries
            intersects = False
            for test_geometry in test_geometries:
                intersects |= polygon.intersects(test_geometry)
                if intersects:
                    break

            # The polygon is good: add the velocity vector and create an obstacle
            if not intersects:
                # Speed = 0.5 (from 0 to 1) in a random direction
                vel = (random.randint(0, 2) - 1 * 0.5, random.randint(0, 2) - 1 * 0.5, random.randint(0, 2) - 1 * 0.5)

                obstacle = Obstacle(polygon, pose, vel)
                obstacles.append(obstacle)

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
