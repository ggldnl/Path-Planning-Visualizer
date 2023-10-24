from model.world.map.map_builder import MapBuilder
import model.geometry.utils as utils
from model.exceptions.collision_exception import CollisionException


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

    def add_robot(self, robot, controller):
        self.robots.append(robot)
        self.controllers.append(controller)

    def solids(self):
        return self.map.obstacles + self.robots

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt

        """
        for controller in self.controllers:
            controller.step_motion(self.map)
        """

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

        solids = self.solids()

        for robot in self.robots:

            polygon1 = robot.body

            # Robots can collide with other robots and with the obstacles
            for solid in solids:

                if solid is not robot:  # Don't bother testing an object against itself

                    polygon2 = solid.polygon  # polygon2

                    if utils.check_nearness(
                        polygon1, polygon2
                    ):  # Don't bother testing objects that are not near each other
                        if polygon1.intersects(polygon2):
                            raise CollisionException(f'Robot {robot.name} collided with an obstacle.')
