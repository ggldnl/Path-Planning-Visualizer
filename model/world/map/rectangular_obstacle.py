from model.world.map.obstacle import Obstacle
from model.geometry.polygon import Polygon
from model.geometry.point import Point


class RectangularObstacle(Obstacle):

    def __init__(self, width, height, pose, vel=None):

        if width <= 0:
            raise ValueError(f'Invalid width: {width}')

        if height <= 0:
            raise ValueError(f'Invalid height: {height}')

        polygon = Polygon([
            Point(0, 0),
            Point(0, height),
            Point(width, height),
            Point(width, 0)
        ])

        super().__init__(polygon, pose, vel)


