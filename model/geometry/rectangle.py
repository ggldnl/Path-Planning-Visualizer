from model.geometry.point import Point
from model.geometry.polygon import Polygon


class Rectangle(Polygon):

    def __init__(self, width, height):

        if width <= 0:
            raise ValueError(f'Invalid width: {width}')

        if height <= 0:
            raise ValueError(f'Invalid height: {height}')

        points = [
            Point(0, 0),
            Point(0, height),
            Point(width, height),
            Point(width, 0)
        ]

        super().__init__(points)
