from model.controllers_legacy.controller import Controller

from model.geometry.point import Point


class DummyController(Controller):

    def __init__(self, robot, map, iterations=1, discretization_step=0.2):

        super().__init__(robot, map, iterations, discretization_step)

        self.path_backup = []
        self.idx = 0

        self._init()

    def _init(self):
        # Zig zag
        self.path_backup = [

            # Describe a square and go back to the starting point
            Point(0, 0),
            Point(0, 1),
            Point(1, 1),
            Point(1, 0),
            Point(0, 0),

            # Go to the goal
            self.map.goal.copy(),
        ]
        self.idx = 0

    def _search(self):
        if not self.has_path():
            # Do nothing: the dummy controller always return the same path. We only reset the path
            self.path.append(self.path_backup[self.idx])
            print(f'Adding {self.path[self.idx]} to ne path')
            self.idx += 1
        else:
            self.idx = 0
