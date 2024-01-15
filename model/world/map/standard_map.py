from model.geometry.intersection import check_intersection

from model.world.map.map import Map


class StandardMap(Map):

    def __init__(self, **kwargs):
        """
        This implementation of the Map interface only relies on a simple list to
        manage the obstacles. This results in very slow queries and overall latency
        """

        super().__init__(**kwargs)

    def query_polygon(self, region):
        result = []
        for obstacle_id, obstacle in self._obstacles.items():
            if check_intersection(region, obstacle.polygon):
                result.append(obstacle_id)
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

    def _add_obstacle(self, obstacle):
        return

    def _remove_obstacle(self, obstacle_id):
        return

    def _reset(self):
        return

    def _clear(self):
        return
