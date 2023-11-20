# Math
from model.geometry.polygon import Polygon
from rtree import index

# Map
from model.world.map.map import Map


class SpatialMap(Map):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.obstacles = []
        self.obstacles_index = index.Index()

        self.initial_obstacles = []

    def _add_obstacle(self, obstacle):

        bounds = obstacle.polygon.get_bounding_box()
        self.obstacles_index.insert(len(self.obstacles), bounds)
        self.obstacles.append(obstacle)
        # Implicitly increment the id

        self.initial_obstacles.append(obstacle.copy())

    def step_motion(self, dt):
        for obstacle_id in range(len(self.obstacles)):
            obstacle = self.obstacles[obstacle_id]
            bounding_box = obstacle.polygon.get_bounding_box()
            obstacle.step_motion(dt)
            self.obstacles_index.delete(obstacle_id, bounding_box)
            self.obstacles_index.delete(obstacle_id, obstacle.polygon.get_bounding_box())

    def reset_map(self):

        self.obstacles = []
        for obstacle in self.initial_obstacles:
            bounds = obstacle.polygon.get_bounding_box()
            self.obstacles_index.insert(len(self.obstacles), bounds)
            self.obstacles.append(obstacle)

    def query_region(self, region: Polygon):

        # Assuming region is a Polygon representing the query region
        result = []
        for obj_id in self.obstacles_index.intersection(region.get_bounding_box()):

            # Check if the actual geometry intersects with the query region
            if region.intersects(self.get_polygon(obj_id)):
                result.append(obj_id)

        return result

    def get_polygon(self, obj_id):
        # Retrieve the polygon geometry based on its identifier
        return self.obstacles[obj_id]

    def save_as_pickle(self, filename):
        pass

    def save_as_json(self, filename):
        pass

    def load_map_from_pickle(self, filename):
        pass

    def load_map_from_json_data(self, data):
        pass
