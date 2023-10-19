from model.world.map.map import Map


# Default environment parameters
OBS_MIN_WIDTH = 0.2  # meters
OBS_MAX_WIDTH = OBS_MIN_WIDTH + 0.5
OBS_MIN_HEIGHT = 0.4
OBS_MAX_HEIGHT = OBS_MIN_HEIGHT + 0.5
OBS_MIN_DIST = 0.8
OBS_MAX_DIST = 5.0
OBS_STEADY_COUNT = 20
OBS_MOVING_COUNT = 20
OBS_MIN_LIN_SPEED = -0.1
OBS_MAX_LIN_SPEED = 0.1
OBS_MIN_ANG_SPEED = -45
OBS_MAX_ANG_SPEED = 45
GOAL_MIN_DIST = 2.0
GOAL_MAX_DIST = 3.0
MIN_GOAL_CLEARANCE = 0.1


class MapBuilder:

    def __init__(self):

        self.map = Map(

            # Obstacle parameters
            obs_min_dist=OBS_MIN_DIST,
            obs_max_dist=OBS_MAX_DIST,

            # If the obstacles are rectangles:
            obs_min_width=OBS_MIN_WIDTH,
            obs_max_width=OBS_MAX_WIDTH,
            obs_min_height=OBS_MIN_HEIGHT,
            obs_max_height=OBS_MAX_HEIGHT,

            obs_steady_count=OBS_STEADY_COUNT,
            obs_moving_count=OBS_MOVING_COUNT,

            # Speed
            obs_min_lin_speed=OBS_MIN_LIN_SPEED,
            obs_max_lin_speed=OBS_MAX_LIN_SPEED,
            obs_min_ang_speed=OBS_MIN_ANG_SPEED,
            obs_max_ang_speed=OBS_MAX_ANG_SPEED,

            # Goal parameters
            goal_min_dist=GOAL_MIN_DIST,
            goal_max_dist=GOAL_MAX_DIST,

            min_goal_clearance=MIN_GOAL_CLEARANCE
        )

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
        self.map.obs_min_dist = obs_min_dist
        self.map.obs_max_dist = obs_max_dist
        return self

    def set_obs_min_width_range(self, obs_min_width, obs_max_width):
        self._check_range(obs_min_width, obs_max_width)
        self.map.obs_min_width = obs_min_width
        self.map.obs_max_width = obs_max_width
        return self

    def set_obs_min_height_range(self, obs_min_height, obs_max_height):
        self._check_range(obs_min_height, obs_max_height)
        self.map.obs_min_height = obs_min_height
        self.map.obs_max_height = obs_max_height
        return self

    def set_obs_steady_count(self, obs_steady_count):
        self._check_non_negative(obs_steady_count, strict=False)  # could be zero if we don't want steady obstacles
        self.map.obs_steady_count = obs_steady_count
        return self

    def set_obs_moving_count(self, obs_moving_count):
        self._check_non_negative(obs_moving_count, strict=False)
        self.map.obs_moving_count = obs_moving_count

    def set_obs_linear_speed_range(self, obs_min_lin_speed, obs_max_lin_speed):
        self._check_range(obs_min_lin_speed, obs_max_lin_speed)
        self.map.obs_min_lin_speed = obs_min_lin_speed
        self.map.obs_max_lin_speed = obs_max_lin_speed
        return self

    def set_obs_angular_speed_range(self, obs_min_ang_speed, obs_max_ang_speed):
        self._check_range(obs_min_ang_speed, obs_max_ang_speed)
        self.map.obs_min_ang_speed = obs_min_ang_speed
        self.map.obs_max_ang_speed = obs_max_ang_speed
        return self

    def set_goal_dist_range(self, goal_min_dist, goal_max_dist):
        self._check_range(goal_min_dist, goal_max_dist)
        self.map.goal_min_dist = goal_min_dist
        self.map.goal_max_dist = goal_max_dist
        return self

    def build(self):
        return self.map
