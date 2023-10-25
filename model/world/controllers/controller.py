from abc import ABCMeta, abstractmethod
from ..map.map import Map


class Controller(metaclass=ABCMeta):
    # si interfaccia con il mondo
    def __init__(self, robot):
        self.robot = robot
        #self.drive_train # vettore da generare dove deve andare robot

    def get_current_state(self, map: Map):
        goal_pos = map.goal
        return goal_pos, self.robot.estimated_pose

    @abstractmethod
    def step_motion(self, map: Map):
        return
