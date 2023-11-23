from model.world.controllers.controller import Controller

from model.geometry.point import Point
from model.geometry.segment import Segment


class DummyController(Controller):

    def __init__(self, robot, map):

        super().__init__(robot, map)

    def search(self):

        # Do nothing: the dummy controller always return the same path. We only reset the path

        # Zig zag
        self.path = [
            Point(3, 0),
            Point(3, 1),
            Point(2, 0),
            Point(2, 1),
            Point(1, 0),
            Point(1, 1),
            Point(0, 0)
        ]

    def reset(self):
        self.path = []

    def populate_draw_list(self):
        # For A* the segments we need to draw are the segments in self.path
        self.draw_list = [Segment(self.path[i - 1], self.path[i]) for i in range(1, len(self.path))]
