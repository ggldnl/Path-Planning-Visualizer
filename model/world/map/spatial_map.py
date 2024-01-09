
# Model
from model.geometry.intersection import check_intersection
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

        # Current obstacles position
        self._obstacles_tree = index.Index()

    def add_obstacle(self, obstacle):
        self._obstacles_tree.insert(len(self._obstacles), obstacle.polygon.bounds)
        self._obstacles.append(obstacle)
        self._initial_obstacles.append(obstacle.copy())

    def step_motion(self, dt):
        # TODO spatial map is not thread safe, flask requires multi threading

        """
        if self.allow_changes:

            for obstacle_id in range(len(self._obstacles)):
                obstacle = self._obstacles[obstacle_id]
                bounds = obstacle.polygon.bounds

                if self.boundaries:

                    # Try to compute the next pose
                    x, y, z = obstacle.polygon.pose
                    vx, vy, vz = obstacle.vel
                    lsm = obstacle.linear_speed_multiplier

                    new_x = x + vx * lsm * dt
                    new_y = y + vy * lsm * dt

                    if not -self.obs_max_dist <= new_x <= self.obs_max_dist:
                        obstacle.vel = (-vx, vy, vz)

                    if not -self.obs_max_dist <= new_y <= self.obs_max_dist:
                        obstacle.vel = (vx, -vy, vz)

                obstacle.step_motion(dt)

                self._obstacles_tree.delete(obstacle_id, bounds)
                self._obstacles_tree.insert(obstacle_id, obstacle.polygon.bounds)
        """
        pass

    def reset_map(self):
        self._obstacles = []
        for obstacle in self._initial_obstacles:
            bounds = obstacle.polygon.get_bounding_box()
            self._obstacles_tree.insert(len(self._obstacles), bounds)
            self._obstacles.append(obstacle)

    def query_region(self, region: Polygon):
        # Assuming region is a Polygon representing the query region
        result = []
        for obj_id in self._obstacles_tree.intersection(region.bounds):

            # Check if the actual geometry intersects with the query region
            if check_intersection(region, self.get_polygon(obj_id)):
                result.append(obj_id)

        return result

    def query(self, bounds):
        return self._obstacles_tree.intersection(bounds)

    def clear(self):
        self._obstacles = []
        self._initial_obstacles = []
        self._obstacles_tree = index.Index()

    def save_as_pickle(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self._initial_obstacles, file)
            pickle.dump(self._current_goal, file)

    def save_as_json(self, filename):
        data = {
            "initial_obstacles": [obstacle.to_dict() for obstacle in self._initial_obstacles],
            "current_goal": self._current_goal.to_dict()
        }

        with open(filename, "w") as file:
            json.dump(data, file)

    def save_map(self, filename):
        self.save_as_json(filename)

    def load_map_from_pickle(self, filename):
        with open(filename, "rb") as file:
            self._initial_obstacles = pickle.load(file)
            self._obstacles = [obstacle.copy() for obstacle in self._initial_obstacles]
            self._current_goal = pickle.load(file)

    def load_map_from_json_file(self, filename):
        with open(filename, 'rb') as file:
            data = json.load(file)
            self.load_map_from_json_data(data)

    def load_map_from_json_data(self, data):
        self._current_goal = Point.from_dict(data['current_goal'])

        # reset the current obstacle if present
        self._obstacles = []
        self._obstacles_tree = index.Index()

        obstacle_data = data['initial_obstacles']
        for obstacle_dictionary in obstacle_data:
            obstacle = Obstacle.from_dict(obstacle_dictionary)
            self.add_obstacle(obstacle)

    def load_map(self, filename):
        self.load_map_from_json_file(filename)
