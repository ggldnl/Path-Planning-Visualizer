from abc import abstractmethod

import numpy as np

# Serialization
import json
import pickle

# Local imports
from model.geometry.point import Point
from model.geometry.circle import Circle
from model.geometry.polygon import Polygon
from model.geometry.rectangle import Rectangle
from model.geometry.intersection import check_intersection

from model.world.map.obstacle import Obstacle


class Map:
    """
    The map should be generated first. Once generated, a goal and some obstacles
    are added to it. Each obstacle has an ID and a dictionary containing
    (obstacle_id: obstacle) key:value pairs is maintained in the Map interface.
    The logic to efficiently query the obstacles space is implemented in the
    subclasses (e.g. we can use a simple list to keep things simple or a quad
    tree to efficiently retrieve all the obstacles in a particular region).
    Queries on the obstacle space for a region (specified either by a bounds or
    by a polygon) return the IDs of the obstacles that intersect the region.
    """

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
        self._initial_obstacles = {}

        # Current obstacles
        self._obstacles = {}

        # Obstacle ID
        self._next_obstacle_id = 0

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
        return list(self._obstacles.values())

    def set_goal(self, goal, clearance=0.2):
        """
        Set a new goal only if there are no obstacles near it
        """
        if (self.map_boundaries[0] < goal.x < self.map_boundaries[2] and
                self.map_boundaries[1] < goal.y < self.map_boundaries[3]):

            obstacles_near_new_goal = self.query_polygon(Circle(goal.x, goal.y, clearance))
            if len(obstacles_near_new_goal) == 0:
                self._current_goal = goal
                return True
            return False
        return False

    def add_obstacle(self, obstacle):

        if self.goal is None:
            raise RuntimeError('Map has not a goal yet!')

        # If we enabled changes to the map
        if self.enable_changes:

            # If the obstacle is inside the map boundaries
            if (self.map_boundaries[0] < obstacle.polygon.pose.x < self.map_boundaries[2] and
                    self.map_boundaries[1] < obstacle.polygon.pose.y < self.map_boundaries[3]):

                # If there are no obstacles near the new one
                # if not obstacle.polygon.check_nearness(Circle(self.goal.x, self.goal.y, self.goal_min_clearance)):
                if len(self.query_polygon(Circle(self.goal.x, self.goal.y, 0.1))) == 0:
                    obstacle_id = self._next_obstacle_id
                    self._obstacles[obstacle_id] = obstacle

                    # Call to the private method
                    self._add_obstacle(obstacle)

                    # Increment the index for the next polygon
                    self._next_obstacle_id += 1

                    return obstacle_id

        return -1

    @abstractmethod
    def _add_obstacle(self, obstacle):
        """
        We can use complex data structures to efficiently query the obstacle
        space. This means we need to maintain other data structures in the
        subclasses and update them when adding an obstacle
        """
        pass

    def remove_obstacle(self, obstacle_id):

        if self.enable_changes:

            if obstacle_id in self._obstacles:
                del self._obstacles[obstacle_id]

                # Update other data structures
                self._remove_obstacle(obstacle_id)

                return True

        return False

    @abstractmethod
    def _remove_obstacle(self, obstacle_id):
        pass

    def add_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    def spawn_obstacle_at(self, point):
        """
        Generates a random obstacle centered in the specified point
        """
        polygon = self._generate_random_polygon(point)
        return self.add_obstacle(Obstacle(polygon))

    def enable(self):
        self.enable_changes = True

    def disable(self):
        self.enable_changes = False

    def query_bounds(self, bounds):
        """
        Query a region defined by (minx, miny, maxx, maxy)
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

    def reset(self):
        """
        Reset the map by recovering the initial state of the obstacles.
        We may want to reset other data structures too, that is why we call
        the abstract _reset method
        """
        self._obstacles = self._initial_obstacles.copy()
        self._next_obstacle_id = max(self._obstacles.keys(), default=0) + 1
        self._reset()

    @abstractmethod
    def _reset(self):
        pass

    def clear(self):
        """
        Clear the map by removing everything. We may want to clear other data
        structures too, that is why we call the abstract _clear method
        """
        self._obstacles = {}
        self._next_obstacle_id = 0
        self._clear()

    @abstractmethod
    def _clear(self):
        pass

    def to_dict(self):
        return {
            "obstacles": [{'id': oid, 'obstacle': o.to_dict()} for oid, o in self._obstacles.items()],
            "goal": self._current_goal.to_dict()
        }

    # ------------------------------------ IO ------------------------------------ #

    def load_map(self, filename):
        self.load_from_json(filename)

    def save_map(self, filename):
        self.save_as_json(filename)

    def save_as_pickle(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self, file)

    def load_from_pickle(self, filename):
        with open(filename, 'rb') as file:
            obj = pickle.load(file)
            self._initial_obstacles = obj._initial_obstacles.copy()
            self._obstacles = obj._obstacles.copy()
            self._next_obstacle_id = max(self._obstacles.keys(), default=0) + 1
            self._current_goal = obj._current_goal
            self._load_from_pickle()

    @abstractmethod
    def _load_from_pickle(self):
        pass

    def load_from_json(self, filename):
        with open(filename, 'rb') as file:
            data = json.load(file)
            self.load_from_json_data(data)

    def save_as_json(self, filename):
        data = self.to_dict()

        with open(filename, "w") as file:
            json.dump(data, file)

    def load_from_json_data(self, data):
        self._current_goal = Point.from_dict(data['goal'])
        self._obstacles = {o_dict['id']: Obstacle.from_dict(o_dict['obstacle']) for o_dict in data['obstacles']}
        self._initial_obstacles = self._obstacles.copy()
        self._next_obstacle_id = max(self._obstacles.keys(), default=0) + 1
        self._load_from_json_data()

    @abstractmethod
    def _load_from_json_data(self):
        pass

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
        self._obstacles = {oid: o for oid, o in enumerate(obstacles)}
        self._initial_obstacles = self._obstacles.copy()
        self._next_obstacle_id = len(obstacles)
        self._current_goal = goal
