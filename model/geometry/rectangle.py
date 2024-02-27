from model.geometry.point import Point
from model.geometry.polygon import Polygon
import numpy as np


class Rectangle(Polygon):

    def __init__(self, width, height):

        if width <= 0:
            raise ValueError(f'Invalid width: {width}')

        if height <= 0:
            raise ValueError(f'Invalid height: {height}')

        points = [
            Point(-width/2, -height/2),
            Point(-width/2, height/2),
            Point(width/2, height/2),
            Point(width/2, -height/2)
        ]

        super().__init__(points)
