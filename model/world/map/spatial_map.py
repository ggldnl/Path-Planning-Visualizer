# Math
from math import pi, sin, cos
import random

from model.geometry.intersection import check_intersection
# Model
from model.geometry.rectangle import Rectangle
from model.geometry.segment import Segment
from model.world.map.map import Map
from model.world.map.obstacle import Obstacle
from model.geometry.polygon import Polygon
from model.geometry.point import Point

# Serialization
import pickle
import json

from rtree import index


class SpatialMap(Map):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Current obstacle position
        self._obstacles = []
        self._obstacles_tree = index.Index()

        # Initial obstacle position
        self._initial_obstacles = []

    @property
    def goal(self):
        return self.current_goal

    @property
    def obstacles(self):
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacle):
        self._obstacles = obstacle

    def _add_obstacle(self, obstacle):
        self._obstacles_tree.insert(len(self._obstacles), obstacle.polygon.bounds)
        self._obstacles.append(obstacle)
        self._initial_obstacles.append(obstacle.copy())

    def step_motion(self, dt):
        for obstacle_id in range(len(self._obstacles)):
            obstacle = self._obstacles[obstacle_id]
            bounds = obstacle.polygon.bounds
            obstacle.step_motion(dt)
            self._obstacles_tree.delete(obstacle_id, bounds)
            self._obstacles_tree.insert(obstacle_id, obstacle.polygon.bounds)

    def reset_map(self):
        self._obstacles = []
        for obstacle in self._initial_obstacles:
            bounds = obstacle.polygon.get_bounding_box()
            self._obstacles_tree.insert(len(self._obstacles), bounds)
            self._obstacles.append(obstacle)

    def check_collision(self, node1, node2):
        line = Segment(node1, node2)
        for obstacle_id in self._obstacles_tree.intersection(line.bounds):
            if check_intersection(line, self.get_polygon(obstacle_id)):
                return True
        return False

    def get_neighbors(self, node, step_size=1, decimal_places=1):
        if len(node) == 3:
            x, y, z = node
        elif len(node) == 2:
            x, y = node
        neighbors = [
            (round(x - step_size, decimal_places), round(y, decimal_places)),
            (round(x + step_size, decimal_places), round(y, decimal_places)),
            (round(x, decimal_places), round(y - step_size, decimal_places)),
            (round(x, decimal_places), round(y + step_size, decimal_places)),
            (round(x - step_size, decimal_places), round(y - step_size, decimal_places)),
            (round(x + step_size, decimal_places), round(y - step_size, decimal_places)),
            (round(x - step_size, decimal_places), round(y + step_size, decimal_places)),
            (round(x + step_size, decimal_places), round(y + step_size, decimal_places)),
        ]
        return neighbors

    def query_region(self, region: Polygon):
        # Assuming region is a Polygon representing the query region
        result = []
        for obj_id in self._obstacles_tree.intersection(region.get_bounding_box()):

            # Check if the actual geometry intersects with the query region
            if check_intersection(region, self.get_polygon(obj_id)):
                result.append(obj_id)

        return result

    def get_polygon(self, obj_id):
        # Retrieve the polygon geometry based on its identifier
        return self.obstacles[obj_id].polygon

    def save_as_pickle(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self._initial_obstacles, file)
            pickle.dump(self.current_goal, file)

    def save_as_json(self, filename):
        data = {
            "initial_obstacles": [obstacle.to_dict() for obstacle in self._initial_obstacles],
            "current_goal": self.current_goal.to_dict()
        }

        with open(filename, "w") as file:
            json.dump(data, file)

    def save_map(self, filename):
        self.save_as_json(filename)

    def load_map_from_pickle(self, filename):
        with open(filename, "rb") as file:
            self._initial_obstacles = pickle.load(file)
            self._obstacles = [obstacle.copy() for obstacle in self._initial_obstacles]
            self.current_goal = pickle.load(file)

    def load_map_from_json_file(self, filename):
        with open(filename, 'rb') as file:
            data = json.load(file)
            self.load_map_from_json_data(data)

    def load_map_from_json_data(self, data):
        self.current_goal = Point.from_dict(data['current_goal'])

        # reset the current obstacle if present
        self._obstacles = []
        self._obstacle_tree = index.Index()

        obstacle_data = data['initial_obstacles']
        for obstacle_dictionary in obstacle_data:
            obstacle = Obstacle.from_dict(obstacle_dictionary)
            self._add_obstacle(obstacle)
        print('Map updated!')

    def load_map(self, filename):
        self.load_map_from_json_file(filename)
