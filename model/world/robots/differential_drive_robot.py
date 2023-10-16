from .robot import Robot


class DifferentialDriveRobot(Robot):

    def __init__(self):
        super().__init__("Differential Drive")

    def step_motion(self,
                    velocity_vector: tuple):

        return