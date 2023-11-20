from model.world.map.map_builder import MapBuilder
from model.exceptions.collision_exception import CollisionException
from model.geometry.intersection import check_intersection


class World:

    def __init__(self, dt):

        # Initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds

        self.robots = []

        # Initialize lists of world objects
        self.controllers = []

        # Initialize the map_legacy
        self.map = MapBuilder().set_obs_moving_count(20).set_obs_steady_count(20).build()

        # TODO bug: map is generated before we add robots; we can add robots over obstacles
        self.map.generate(self.robots)
        # self.map_legacy.load_map(r'/home/daniel/Git/Robot-Simulator/model/world/map_legacy/maps/map_test.json')

    def set_period(self, dt):
        self.dt = dt

    def add_robot(self, robot, controller):
        self.robots.append(robot)
        self.controllers.append(controller)

    def reset_robots(self):
        for robot in self.robots:
            robot.pose.x = 0
            robot.pose.y = 0
            robot.pose.z = 0

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt

        """
        for controller in self.controllers:
            controller.step_motion(self.map_legacy)
        """

        # Step all the obstacles
        # TODO setup controllers
        for obstacle in self.map.obstacles:
            obstacle.step_motion(dt)

        for robot in self.robots:
            robot.step_motion(dt)

        # Increment world time
        self.world_time += dt

    def apply_physics(self):
        self._detect_collisions()

    def _detect_collisions(self):
        """
        Test the world for existing collisions with solids.
        Raises a CollisionException if one occurs.
        """

        solids = self.map.obstacles + self.robots

        for robot in self.robots:

            polygon1 = robot.outline

            # Robots can collide with other robots and with the obstacles
            for solid in solids:

                if solid is not robot:  # Don't bother testing an object against itself

                    polygon2 = solid.polygon  # polygon2

                    # Don't bother testing objects that are not near each other
                    if polygon1.check_nearness(polygon2):
                        if check_intersection(polygon1, polygon2):
                            raise CollisionException(f'Robot {robot.name} collided with an obstacle.')
