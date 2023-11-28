from model.world.map.map import Map
from model.geometry.point import Point
from model.geometry.segment import Segment
from model.geometry.polygon import Polygon
from model.world.map.obstacle import Obstacle
from model.geometry.intersection import check_intersection

import pickle
import json


class StandardMap(Map):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
    def _add_obstacle(self, obstacle):
        self._obstacles.append(obstacle)
        self._initial_obstacles.append(obstacle.copy())

    def step_motion(self, dt):
        for obstacle in self._obstacles:
            obstacle.step_motion(dt)

    def reset_map(self):
        self._obstacles = [obstacle.copy() for obstacle in self._initial_obstacles]

    def clear(self):
        self._obstacles = []
        self._initial_obstacles = []

    def query_region(self, region):
        result = []
        for i in range(len(self._obstacles)):
            if check_intersection(region, self._obstacles[i].polygon):
                result.append(i)
        return result

    def query(self, bounds):
        """
        Add some junk just to keep this version of the map compatible, it is too slow and we can't use it anyway
        """
        min_x, min_y, max_x, max_y = bounds
        return self.query_region(Polygon([
            Point(min_x, min_y),
            Point(min_x, max_y),
            Point(max_x, max_y),
            Point(max_x, min_y)
        ]))


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

    def load_map_from_pickle(self, filename):
        with open(filename, "rb") as file:
            self._initial_obstacles = pickle.load(file)
            self._obstacles = [obstacle.copy() for obstacle in self._initial_obstacles]
            self._current_goal = pickle.load(file)

    def load_map_from_json_data(self, data):

        self._current_goal = Point.from_dict(data['current_goal'])

        obstacle_data = data['initial_obstacles']
        obstacles = []
        for obstacle_dictionary in obstacle_data:
            obstacle = Obstacle.from_dict(obstacle_dictionary)
            obstacles.append(obstacle)

        self._obstacles = obstacles
        self._initial_obstacles = [obstacle.copy() for obstacle in self._obstacles]
        print('Map updated!')

    """
    def check_collision(self, point1, point2):
        # line = Segment(point1, point2)
        # buffer = Polygon.get_segment_buffer(line, self.discretization_step, self.discretization_step)
        buffer = Polygon.get_point_buffer(point2, self.discretization_step)
        for obstacle_id in range(len(self._obstacles)):
            if check_intersection(buffer, self.get_polygon(obstacle_id)):
                return True
        return False
    """
