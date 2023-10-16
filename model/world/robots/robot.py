from abc import ABCMeta, abstractmethod
from .differential_drive_robot import DifferentialDriveRobot
from .non_holonomic_robot import NonHolonomicRobot
from ..motor.motor import Motor
from ..sensors.sensor import Sensor

from model.geometry.polygon import Polygon
from model.geometry.pose import Pose
from shapely import geometry


class RobotFactory:
    @staticmethod
    def create_robot(name, robot_type):
        if robot_type == "DifferentialDrive":
            return Robot(name, DifferentialDriveRobot())
        elif robot_type == "Hexapod":
            return Robot(name, NonHolonomicRobot())
        else:
            raise ValueError("Unsupported robot type")


class Robot(metaclass=ABCMeta):
    def __init__(self,
                 name: str,
                 body: Polygon, ):
        self.name = name
        self.sensors = []

        self.estimated_pose = Pose(x=0,
                                   y=0,
                                   theta=0)  # pos in the world
        self.body = body  # geometry
        self.sensors = []
        self.sensors_pose = []  # relative to the robot
        self.motors = []
        self.motors_pose = []  # relative to the robot

    def add_sensor(self,
                   sensor: Sensor,
                   pose: Pose):
        self.sensors.append(sensor)
        self.sensors_pose.append(pose)

    def add_motor(self,
                  motor: Motor,
                  pose: Pose):
        self.sensors.append(motor)
        self.sensors_pose.append(pose)

    def read_all_sensors(self):
        sensor_data = {}
        for sensor in self.sensors:
            sensor_data[sensor.name] = sensor.read()
        return sensor_data

    @abstractmethod
    def step_motion(self, velocity_vector: tuple):
        pass

    def apply_dynamics(self, x_vector, y_vector, theta):
        # dato il vettore velocita calcola quanto si è mosso
        return

    def detect_collision(self):
        pass

    def _update_sensors(self):
        for sensor in self.sensors:
            sensor.read()