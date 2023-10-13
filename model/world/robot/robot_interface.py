class RobotInterface:

    def __init__(self, polygons):

        # The base can be composed by multiple polygons,
        # differently from the obstacles (single polygon)
        self.polygons = polygons

        self.motors = []
        self.motor_pose = []

        self.sensors = []
        self.sensors_pose = []

    def step_motion(self, dt):
        pass
