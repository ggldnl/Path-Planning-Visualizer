from model.world.map.map_builder import MapBuilder
from random import Random


class World:

    def __init__(self, dt):

        # Initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds

        self.robots = []

        # Initialize lists of world objects
        self.controllers = []

        # Initialize the map
        self.map = MapBuilder().build()

        # TODO provide load map capabilities
        self.map.get_map(self.robots)

    def set_period(self, dt):
        self.dt = dt

    def add_robot_controller(self, controller, robot):
        self.controllers.append(controller(robot))

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt

        for controller in self.controllers:
            controller.step_motion(self.map)


        # Step all the obstacles
        for obstacle in self.map.obstacles:
            obstacle.step_motion(dt)

        # Apply physics interactions
        self._apply_physics()

        # Increment world time
        self.world_time += dt


    def _apply_physics(self):
        self._detect_collisions()



    def _detect_collisions(self):
        """
        Test the world for existing collisions with solids.
        Raises a CollisionException if one occurs.
        """
        pass
