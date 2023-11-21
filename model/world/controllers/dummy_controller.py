from model.world.controllers.controller import Controller

from model.geometry.point import Point


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
