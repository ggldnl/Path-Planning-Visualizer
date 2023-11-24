from model.world.controllers.controller import Controller

from model.geometry.point import Point
from model.geometry.segment import Segment


class DummyController(Controller):

    def __init__(self, robot, map):

        super().__init__(robot, map)

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

    def search(self):
        print('Searching...')
        if not self.has_path():
            # Do nothing: the dummy controller always return the same path. We only reset the path
            self.path.append(self.path_backup[self.idx])
            print(f'Adding {self.path[self.idx]} to ne path')
            self.idx += 1
        else:
            self.idx = 0

    def reset(self):
        # Nothing to reset, the path is emptied one point at a time once the robot reaches it
        self.path = []
        self.path_backup[-1] = self.map.goal.copy()
        self.idx = 0

    def get_draw_list(self):
        return self.map.get_neighbors(self.robot.current_pose.as_point())
