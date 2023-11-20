from model.world.map.map import Map
from model.geometry.point import Point
from model.world.map.obstacle import Obstacle

import pickle
import json


class StandardMap(Map):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.obstacles = []
        self.initial_obstacles = []

    def _add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
        self.initial_obstacles.append(obstacle.copy())

    def reset_map(self):
        self.obstacles = [obstacle.copy() for obstacle in self.initial_obstacles]

    def save_as_pickle(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self.initial_obstacles, file)
            pickle.dump(self.current_goal, file)

    def save_as_json(self, filename):
        data = {
            "initial_obstacles": [obstacle.to_dict() for obstacle in self.initial_obstacles],
            "current_goal": self.current_goal.to_dict()
        }

        with open(filename, "w") as file:
            json.dump(data, file)

    def load_map_from_pickle(self, filename):
        with open(filename, "rb") as file:
            self.initial_obstacles = pickle.load(file)
            self.obstacles = [obstacle.copy() for obstacle in self.initial_obstacles]
            self.current_goal = pickle.load(file)

    def load_map_from_json_data(self, data):

        self.current_goal = Point.from_dict(data['current_goal'])

        obstacle_data = data['initial_obstacles']
        obstacles = []
        for obstacle_dictionary in obstacle_data:
            obstacle = Obstacle.from_dict(obstacle_dictionary)
            obstacles.append(obstacle)

        self.obstacles = obstacles
        self.initial_obstacles = [obstacle.copy() for obstacle in self.obstacles]
        print('Map updated!')
