from model.geometry.intersection import check_intersection

from model.world.map.map import Map

from model.world.map.quad_tree import QuadTree


class SpatialMap(Map):

    def __init__(self, **kwargs):
        """
        This implementation of the Map interface uses a quad tree to make spatial queries
        """

        super().__init__(**kwargs)

        self.quad_tree = QuadTree(self.map_boundaries)

    def _add_obstacle(self, obstacle):
        """
        Additional logic to handle the quad tree
        """
        self.quad_tree.insert(self._next_obstacle_id, obstacle.polygon)

    def _remove_obstacle(self, obstacle_id):
        """
        Additional logic to handle the quad tree
        """
        self.quad_tree.remove(obstacle_id)

    def query_polygon(self, polygon):
        result = []
        for obj_id in self.quad_tree.query_region(polygon.get_bounds()):

            # Check if the actual geometry intersects with the query region
            if check_intersection(polygon, self._obstacles[obj_id].polygon):
                result.append(obj_id)

        return result

    def query_bounds(self, bounds):
        """
        Redefine query_bounds to make it more efficient
        """
        return self.quad_tree.query_region(bounds)

    def step_motion(self, dt):
        """
        if self.enable_changes:

            for obstacle in self._obstacles:

                if self.map_boundaries is not None:

                    # Try to compute the next pose
                    x, y, z = obstacle.polygon.pose
                    vx, vy, vz = obstacle.vel
                    lsm = obstacle.linear_speed_multiplier

                    new_x = x + vx * lsm * dt
                    new_y = y + vy * lsm * dt

                    minx, miny, maxx, maxy = self.map_boundaries
                    if not minx <= new_x <= maxx:
                        obstacle.vel = (-vx, vy, vz)

                    if not miny <= new_y <= maxy:
                        obstacle.vel = (vx, -vy, vz)

                obstacle.step_motion(dt)
        """

        # Do nothing for this kind of map, obstacles should stay still
        pass

    def _reset(self):
        self.quad_tree = QuadTree(self.map_boundaries)
        for obstacle_id, obstacle in self._obstacles.items():
            self.quad_tree.insert(obstacle_id, obstacle.polygon)

    def _clear(self):
        self.quad_tree = QuadTree(self.map_boundaries)

    def _restore_from_obstacles_dict(self):
        for obstacle_id, obstacle in self._obstacles.items():
            self.quad_tree.insert(obstacle_id, obstacle.polygon)

    def generate(self, forbidden_zones):
        self.quad_tree.reset()
        super().generate(forbidden_zones)
        self._restore_from_obstacles_dict()

    def _load_from_pickle(self):
        self.quad_tree.reset()
        self._restore_from_obstacles_dict()

    def _load_from_json_data(self):
        self.quad_tree.reset()
        self._restore_from_obstacles_dict()
