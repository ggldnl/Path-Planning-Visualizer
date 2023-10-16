from abc import ABCMeta
from ..map.map import Map


class Controller(metaclass=ABCMeta):
    # si interfaccia con il mondo
    def __init__(self, robot):
        self.robot = robot
        self.drive_train # vettore da generare dove deve andare robot

    def step_motion(self, map: Map):
        self.robot.
        return
