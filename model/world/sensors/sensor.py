from abc import ABCMeta, abstractmethod


class Sensor(metaclass=ABCMeta):
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def read(self):
        # Implement the read method specific to the sensor type
        pass
