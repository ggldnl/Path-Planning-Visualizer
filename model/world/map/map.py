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
import pickle
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
                 min_goal_clearance,

                 obstacles_type,
                 displacement_type,

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

        # Obstacles parameters range
        self.obs_width_range = self.obs_max_width - self.obs_min_width
        self.obs_height_range = self.obs_max_height - self.obs_min_height
        self.obs_dist_range = self.obs_max_dist - self.obs_min_dist

        self.obstacles_type = obstacles_type
        self.displacement_type = displacement_type

        self.boundaries = boundaries

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
    def _add_obstacle(self, obstacle):
        pass

    def _add_obstacles(self, obstacles):
        for obstacle in obstacles:
            self._add_obstacle(obstacle)

    def enable_moving_obstacles(self):
        for idx in range(len(self._obstacles)):
            self._obstacles[idx].vel = self._initial_obstacles[idx].vel

    def disable_moving_obstacles(self):
        for obstacle in self._obstacles:
            obstacle.vel = (0, 0, 0)

    def _generate_random_obstacle(self):

        # TODO implement other obstacle types

        # Generate dimensions
        width = self.obs_min_width + (np.random.random() * self.obs_width_range)
        height = self.obs_min_height + (np.random.random() * self.obs_height_range)

        if self.displacement_type == 'gridlike':
            width = round(width / 0.1) * 0.1
            height = round(height / 0.1) * 0.1

        # Generate position
        dist = self.obs_min_dist + (np.random.random() * self.obs_dist_range)
        phi = -np.pi + (np.random.random() * 2 * np.pi)
        x = dist * np.sin(phi)
        y = dist * np.cos(phi)

        if self.displacement_type == 'gridlike':
            x = round(x, 1)
            y = round(y, 1)

        # Generate orientation
        if self.displacement_type == 'gridlike':
            random_multiple = np.random.randint(0, 3)
            theta = random_multiple * (np.pi / 2)
        elif self.displacement_type == 'random':
            theta = np.random.random() * 2 * np.pi - np.pi
        else:
            raise ValueError(f'Invalid displacement type: {self.displacement_type}')

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
        x = int(dist * np.sin(phi))  # Round x to an integer
        y = int(dist * np.cos(phi))  # Round y to an integer
        goal = Point(x, y)

        # Generate a proximity test geometry for the goal
        r = self.min_goal_clearance
        goal_test_geometry = Circle(x, y, r)

        # test_geometries contains the robots and the goal
        robot_test_geometries = [Circle(r.current_pose.x, r.current_pose.y, self.obs_min_dist) for r in robots]

        # TODO fix this, bug reported above
        if len(robot_test_geometries) == 0:
            robot_test_geometries.append(Circle(0, 0, self.obs_min_dist))

        test_geometries = robot_test_geometries + [goal_test_geometry]

        # Generate obstacles
        moving_obstacles = []
        while len(moving_obstacles) < self.obs_moving_count:

            polygon = self._generate_random_obstacle()

            # Check if the polygon intersects one of the test geometries
            intersects = False
            for test_geometry in test_geometries:
                intersects |= check_intersection(polygon, test_geometry)
                if intersects:
                    break

            # The polygon is good: add the velocity vector and create an obstacle
            if not intersects:

                obstacle = Obstacle(polygon)

                # Set velocity vector
                obstacle.vel = (np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5))

                if self.displacement_type == 'gridlike':
                    vx = np.random.randint(0, 2) * np.random.uniform(-0.5, 0.5)
                    vy = np.random.uniform(-0.5, 0.5)
                    if vx != 0:
                        vy = 0
                    obstacle.vel = (vx, vy, 0)

                # obstacle.set_random_velocity_vector()

                moving_obstacles.append(obstacle)

        steady_obstacles = []
        while len(steady_obstacles) < self.obs_steady_count:
            
            polygon = self._generate_random_obstacle()

            # Check if the polygon intersects one of the test geometries
            intersects = False
            for test_geometry in test_geometries:
                intersects |= check_intersection(polygon, test_geometry)
                if intersects:
                    break

            # The polygon is good: add the velocity vector and create an obstacle
            if not intersects:

                obstacle = Obstacle(polygon)
                steady_obstacles.append(obstacle)

        # Update the obstacles and the goal
        self._add_obstacles(moving_obstacles)
        self._add_obstacles(steady_obstacles)
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
        # The actual implementation of the map can do other operations other than stepping the obstacles
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
