from abc import ABCMeta


class Sensor(metaclass=ABCMeta):
    def __init__(self, name, sensor_type):
        self.name = name
        self.sensor_type = sensor_type

    def read(self):
        # Implement the read method specific to the sensor type
        pass