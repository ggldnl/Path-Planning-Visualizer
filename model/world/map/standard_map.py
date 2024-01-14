from model.geometry.intersection import check_intersection

from model.world.map.map import Map


class StandardMap(Map):

    def __init__(self, **kwargs):
        """
        This implementation of the Map interface only relies on a simple list to
        manage the obstacles. This results in very slow queries and overall latency
        """

        super().__init__(**kwargs)

    def _can_add(self, new_obstacle):
        for obstacle in self.obstacles:
            if check_intersection(obstacle, new_obstacle):
                return False
            return True

    def add_obstacle(self, obstacle):
        if self._can_add(obstacle.get_bounds()):
            self._obstacles.append(obstacle)

    def spawn_obstacle_at(self, point):
        # TODO
        print('Spawning obstacle PEW PEW!!')

    def query_polygon(self, region):
        result = []
        for i in range(len(self._obstacles)):
            if check_intersection(region, self._obstacles[i].polygon):
                result.append(i)
        return result

    def step_motion(self, dt):

        """
        if self.enable_changes:

            for obstacle in self._obstacles:

                if self.boundaries is not None:

                    # Try to compute the next pose
                    x, y, z = obstacle.polygon.pose
                    vx, vy, vz = obstacle.vel
                    lsm = obstacle.linear_speed_multiplier

                    new_x = x + vx * lsm * dt
                    new_y = y + vy * lsm * dt

                    minx, miny, maxx, maxy = self.boundaries
                    if not minx <= new_x <= maxx:
                        obstacle.vel = (-vx, vy, vz)

                    if not miny <= new_y <= maxy:
                        obstacle.vel = (vx, -vy, vz)

                obstacle.step_motion(dt)
        """

        # Do nothing for this kind of map, obstacles should stay still
        pass

    def reset(self):
        self._obstacles = [obstacle.copy() for obstacle in self._initial_obstacles]

    def clear(self):
        self._obstacles = []
        self._initial_obstacles = []
