from model.world.map.map_builder import MapBuilder
import model.geometry.utils as utils
from model.exceptions.collision_exception import CollisionException


class World:

    def __init__(self, dt):

        # Initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds
        self.idx = 0

        self.robots = []

        # Initialize lists of world objects
        self.controllers = []

        # Initialize the map
        self.map = MapBuilder().set_obs_moving_count(1).set_obs_steady_count(1).build()

        self.map.get_map(self.robots)
        # self.map.load_map(r'/home/daniel/Git/Robot-Simulator/model/world/map/maps/map_test.json')

    def set_period(self, dt):
        self.dt = dt

    def add_robot(self, robot, controller):
        self.robots.append(robot)
        self.controllers.append(controller)

    def reset_robots(self):
        for robot in self.robots:
            robot.pose = (0, 0, 0)

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt

        # Step all the obstacles
        for obstacle in self.map.obstacles:
            obstacle.step_motion(dt)

        for robot, controller in zip(self.robots, self.controllers):
            if self.idx == 0:
                break
            # TODO path.
            if self.idx == 1:
                #path = controller.search()
                pass
            path = controller.search()
            print(path)
            if controller.path:
                print(controller.path)
                robot.apply_dynamics(controller.path.pop())
        self.idx += 1


        # print(self.map.obstacles[0].polygon)
        # print(self.map.obstacles[0].pose)
        # print()

        # Apply physics interactions
        # self._apply_physics()

        # Increment world time
        self.world_time += dt

    def search(self):
        for controller in self.controllers:
            controller.search()  # todo to be sync and not having an instance on controller

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

                    if utils.check_nearness(
                            polygon1, polygon2
                    ):  # Don't bother testing objects that are not near each other
                        if polygon1.intersects(polygon2):
                            raise CollisionException(f'Robot {robot.name} collided with an obstacle.')
