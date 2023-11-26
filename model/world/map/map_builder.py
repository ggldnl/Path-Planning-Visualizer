from model.world.map.map import Map
from model.world.map.standard_map import StandardMap
from model.world.map.spatial_map import SpatialMap
from typing import Literal

default_params = {
    "obs_min_dist": 0.8,
    "obs_max_dist": 5.0,
    "obs_min_width": 0.2,
    "obs_max_width": 0.7,
    "obs_min_height": 0.4,
    "obs_max_height": 0.9,
    "obs_steady_count": 10,
    "obs_moving_count": 60,
    "obs_min_lin_speed": -0.02,
    "obs_max_lin_speed": 0.02,
    "obs_min_ang_speed": -2.0,
    "obs_max_ang_speed": 2.0,
    "goal_min_dist": 4.0,
    "goal_max_dist": 6.0,
    "min_goal_clearance": 0.5,
    "discretization_step": 0.5
}


class MapBuilder:

    def __init__(self):

        self.params_dictionary = default_params
        self.map_type = 'standard'

    @classmethod
    def _check_range(cls, a, b, min_distance=None):
        """
        Checks if the range between a and b is valid and
        optionally if there is at least min_distance between
        the two extrema
        """
        if b >= a:
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

    def set_obs_dist_range(self, obs_min_dist, obs_max_dist):
        self._check_range(obs_min_dist, obs_max_dist)
        self.params_dictionary['obs_min_dist'] = obs_min_dist
        self.params_dictionary['obs_max_dist'] = obs_max_dist
        return self

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

    def set_obs_steady_count(self, obs_steady_count):
        self._check_non_negative(obs_steady_count, strict=False)  # could be zero if we don't want steady obstacles
        self.params_dictionary['obs_steady_count'] = obs_steady_count
        return self

    def set_obs_moving_count(self, obs_moving_count):
        self._check_non_negative(obs_moving_count, strict=False)
        self.params_dictionary['obs_moving_count'] = obs_moving_count
        return self

    def set_obs_linear_speed_range(self, obs_min_lin_speed, obs_max_lin_speed):
        self._check_range(obs_min_lin_speed, obs_max_lin_speed)
        self.params_dictionary['obs_min_lin_speed'] = obs_min_lin_speed
        self.params_dictionary['obs_max_lin_speed'] = obs_max_lin_speed
        return self

    def set_obs_angular_speed_range(self, obs_min_ang_speed, obs_max_ang_speed):
        self._check_range(obs_min_ang_speed, obs_max_ang_speed)
        self.params_dictionary['obs_min_ang_speed'] = obs_min_ang_speed
        self.params_dictionary['obs_max_ang_speed'] = obs_max_ang_speed
        return self

    def set_goal_dist_range(self, goal_min_dist, goal_max_dist):
        self._check_range(goal_min_dist, goal_max_dist)
        self.params_dictionary['goal_min_dist'] = goal_min_dist
        self.params_dictionary['goal_max_dist'] = goal_max_dist
        return self

    def set_discretization_step(self, discretization_step):
        self._check_non_negative(discretization_step, strict=True)
        self.params_dictionary['discretization_step'] = discretization_step
        return self

    def set_type(self, map_type: Literal['standard', 'spatial']):
        self.map_type = map_type
        return self

    def build(self):

        if self.map_type == 'standard':
            map_arch = StandardMap
        else:
            map_arch = SpatialMap

        return map_arch(**self.params_dictionary)
