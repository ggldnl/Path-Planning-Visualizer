from typing import Literal
from model.world.map.standard_map import StandardMap
from model.world.map.spatial_map import SpatialMap
from model.geometry.point import Point
from model.geometry.circle import Circle
from model.geometry.rectangle import Rectangle
from model.geometry.intersection import check_intersection
from model.world.map.obstacle import Obstacle

import numpy as np


default_params = {
    "obs_min_width": 0.2,
    "obs_max_width": 0.7,
    "obs_min_height": 0.4,
    "obs_max_height": 0.9,
    "obs_min_dist": 0.8,
    "obs_max_dist": 5.0,
    "obs_count": 40,
    "goal_min_dist": 3.0,
    "goal_max_dist": 5.0,
    "goal_min_clearance": 0.5,
    "map_boundaries": (-5.0, -5.0, 5.0, 5.0)
}


class MapBuilder:

    def __init__(self):

        self.params_dictionary = default_params

        """
        Map type specifies the data structures used to carry out the computations.
        Standard maps use simple lists, spatial maps use quad trees. Available 
        values are 'list' and 'quadtree' 
        """
        self.data_structure = 'list'

    @classmethod
    def _check_range(cls, a, b, min_distance=None):
        """
        Checks if the range between a and b is valid and
        optionally if there is at least min_distance between
        the two extrema
        """
        if b <= a:
            raise ValueError(f'Invalid range [{a}:{b}]')

        if min_distance is not None:
            if b - a < min_distance:
                raise ValueError(f'Minimum distance between the extrema [{a}:{b}] is not respected')

    @classmethod
    def _check_non_negative(cls, val, strict=False):
        """
        Checks if the value is non-negative (strictly or not)
        """
        if strict:
            if val < 0:
                raise ValueError(f'Value that should be strictly non-negative is negative: {val}')
        else:
            if val <= 0:
                raise ValueError(f'Value that should be non-negative is negative: {val}')

    def set_obs_min_width_range(self, obs_min_width, obs_max_width):
        self._check_range(obs_min_width, obs_max_width)
        self.params_dictionary['obs_min_width'] = obs_min_width
        self.params_dictionary['obs_max_width'] = obs_max_width
        return self

    def set_obs_min_height_range(self, obs_min_height, obs_max_height):
        self._check_range(obs_min_height, obs_max_height)
        self.params_dictionary['obs_min_height'] = obs_min_height
        self.params_dictionary['obs_max_height'] = obs_max_height
        return self

    def set_map_boundaries(self, map_boundaries: tuple[float, float, float, float]):
        self._check_range(map_boundaries[1], map_boundaries[3])
        self._check_range(map_boundaries[0], map_boundaries[2])
        self.params_dictionary['map_boundaries'] = map_boundaries
        return self

    def set_obs_count(self, obs_count):
        self._check_non_negative(obs_count, strict=False)
        self.params_dictionary['obs_count'] = obs_count
        return self

    def set_goal_dist_range(self, goal_min_dist, goal_max_dist):
        self._check_range(goal_min_dist, goal_max_dist)
        self.params_dictionary['goal_min_dist'] = goal_min_dist
        self.params_dictionary['goal_max_dist'] = goal_max_dist
        return self

    def set_goal_clearance(self, goal_min_clearance):
        self._check_non_negative(goal_min_clearance)
        self.params_dictionary['goal_min_clearance'] = goal_min_clearance
        return self

    def set_data_structure(self, data_structure: Literal['list', 'quadtree']):
        self.data_structure = data_structure
        return self

    def build(self):

        if self.data_structure == 'list':
            map_arch = StandardMap
        elif self.data_structure == 'quadtree':
            map_arch = SpatialMap
        else:
            raise ValueError(f'Unsupported map architecture: {self.data_structure}')

        print(self.params_dictionary)

        return map_arch(**self.params_dictionary)
