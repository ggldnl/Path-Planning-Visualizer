import numpy as np

from model.world.sensors.sensor import Sensor
from model.geometry.polygon import Polygon


class ProximitySensor(Sensor):

    def __init__(self,
                 name,
                 max_range,
                 phi_range
                 ):

        super().__init__(name)

        # Sensitivity attributes
        self.max_range = max_range

        self.phi_range = phi_range

        # Compute the base (b) using trigonometry
        h = self.max_range
        b = h * np.tan(self.phi_range)

        # Create a polygon
        self.polygon = Polygon([
            [0, 0],
            [-b / 2, h],
            [b / 2, h]
        ])

        # By default, the read value is infinity
        self.read_value = np.inf

    # Get this sensor's output
    def read(self):
        return self.read_value


