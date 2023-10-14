from model.world.map.map import Map


class World:

    def __init__(self, dt):

        # Initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds

        # Initialize lists of world objects
        self.robots = []

        # Initialize the map
        self.map = Map()

        # TODO provide load map capabilities
        self.map.random_map(self.robots)

    def set_period(self, dt):
        self.dt = dt

    def add_robot(self, robot):
        self.robots.append(robot)

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt

        # Step all the robots
        for robot in self.robots:
            robot.step_motion(dt)

        # Step all the obstacles
        for obstacle in self.map.obstacles:
            obstacle.step_motion(dt)

        # Apply physics interactions
        self._apply_physics()

        # Increment world time
        self.world_time += dt

    def _apply_physics(self):
        self._detect_collisions()
        # TODO: add _update_proximity_sensors()

    def _detect_collisions(self):
        """
        Test the world for existing collisions with solids.
        Raises a CollisionException if one occurs.
        """
        pass
