from .robot import Robot


class NonHolonomicRobot(Robot):

    def __init__(self):
        super().__init__("Non Holonomic Robot")

    def step_motion(self, x_vector, y_vector, angle):
        return