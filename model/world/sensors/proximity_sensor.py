from .sensor import Sensor


class ProximitySensor(Sensor):
    def __init__(self,
                 min_range,
                 max_range,
                 phi_range, #cono rilevamento sensore
                 name,
                 sensor_type):  # view angle of this sensor (rad from front of robot)

        super().__init__(name, sensor_type)

        # sensitivity attributes
        self.min_range = min_range
        self.max_range = max_range
        self.phi_range = phi_range

        #might be useful to implement a minimum and maximum read value?


    # get this sensor's output
    def read(self):
        return self.read_value
