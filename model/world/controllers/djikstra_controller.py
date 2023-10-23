from .controller import Controller
from ..robot.robot import Robot


class DjikstraController(Controller):

    def __init__(self,
                 robot: Robot):
        super().__init__(robot=robot)
