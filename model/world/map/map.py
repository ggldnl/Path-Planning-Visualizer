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

# TODO implement json serialization
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
                 obs_steady_count,
                 obs_moving_count,

                 # Speed
                 obs_min_lin_speed,
                 obs_max_lin_speed,
                 obs_min_ang_speed,
                 obs_max_ang_speed,

                 # Goal parameters
                 goal_min_dist,
                 goal_max_dist,

                 min_goal_clearance
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

        # Number of steady obstacles
        self.obs_steady_count = obs_steady_count

        # Number of moving obstacles
        self.obs_moving_count = obs_moving_count

        # Obstacle speed
        self.obs_min_lin_speed = obs_min_lin_speed
        self.obs_max_lin_speed = obs_max_lin_speed
        self.obs_min_ang_speed = obs_min_ang_speed
        self.obs_max_ang_speed = obs_max_ang_speed

        # Goal distance from the spawning point
        self.goal_min_dist = goal_min_dist
        self.goal_max_dist = goal_max_dist

        self.min_goal_clearance = min_goal_clearance

        # Initial obstacle position
        self.obstacles = []

        # Current obstacle position
        self.current_obstacles = []

        self.current_goal = None

    def get_map(self, robots):

        # Generate the goal
        goal_dist_range = self.goal_max_dist - self.goal_min_dist
        dist = self.goal_min_dist + (random.random() * goal_dist_range)
        phi = -pi + (random.random() * 2 * pi)
        x = dist * sin(phi)
        y = dist * cos(phi)
        goal = Point(x, y)

        # Generate a proximity test geometry for the goal
        r = self.min_goal_clearance
        n = 6
        goal_test_geometry = []
        for i in range(n):
            goal_test_geometry.append(
                Point(x + r * cos(i * 2 * pi / n), y + r * sin(i * 2 * pi / n))
            )
        goal_test_geometry = Polygon(goal_test_geometry)

        # Obstacles parameters range
        obs_width_range = self.obs_max_width - self.obs_min_width
        obs_height_range = self.obs_max_height - self.obs_min_height
        obs_dist_range = self.obs_max_dist - self.obs_min_dist

        # test_geometries contains the robots and the goal
        test_geometries = [r.body for r in robots] + [
            goal_test_geometry
        ]

        # Generate moving obstacles
        obstacles = []
        num_moving_obstacles_generated = 0
        num_steady_obstacles_generated = 0
        while (num_moving_obstacles_generated < self.obs_moving_count or
               num_steady_obstacles_generated < self.obs_steady_count):

            # Generate dimensions
            width = self.obs_min_width + (random.random() * obs_width_range)
            height = self.obs_min_height + (random.random() * obs_height_range)

            # Generate position
            dist = self.obs_min_dist + (random.random() * obs_dist_range)
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

                # If we need to generate moving obstacles
                if num_moving_obstacles_generated < self.obs_moving_count:
                    vel = (
                        random.uniform(self.obs_min_lin_speed, self.obs_max_lin_speed),
                        random.uniform(self.obs_min_lin_speed, self.obs_max_lin_speed),
                        random.uniform(self.obs_min_ang_speed, self.obs_max_ang_speed)
                    )
                    num_moving_obstacles_generated += 1

                # If we are done with the generation of moving obstacles
                else:
                    vel = (0, 0, 0)
                    num_steady_obstacles_generated += 1

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
