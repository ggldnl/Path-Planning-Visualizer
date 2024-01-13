import json

from model.exceptions.collision_exception import CollisionException
from model.geometry.intersection import check_intersection
from model.world import view


class World:

    def __init__(self, world_map, robots, controllers, dt):

        # Initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds
        self.idx = 0

        self.world_map = world_map
        self.robots = robots
        self.controllers = controllers

    @property
    def map(self):
        return self.world_map

    def set_period(self, dt):
        self.dt = dt

    def add_robot(self, robot, controller):
        self.robots.append(robot)
        self.controllers.append(controller)

    def reset_robots(self):
        for robot, controller in zip(self.robots, self.controllers):
            robot.reset()
            controller.reset()

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt
        # Step all the obstacles
        self.world_map.step_motion(dt)

        for robot, controller in zip(self.robots, self.controllers):
            next_pose = controller.step()
            robot.target_pose = next_pose
            robot.step_motion(dt)

        # Apply physics interactions
        # self._apply_physics()

        # Increment world time
        self.world_time += dt

    def _apply_physics(self):
        self._detect_collisions()

    def _detect_collisions(self):
        """
        Test the world for existing collisions with solids.
        Raises a CollisionException if one occurs.
        """

        solids = self.world_map.obstacles + self.robots

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

    def to_dict(self):
        return {
            "map": self.world_map.to_dict(),
            "robots": [robot.to_dict() for robot in self.robots],
            "controllers": [controller.to_dict() for controller in self.controllers],
            "dt": self.dt
        }

    def json_view(self, add_path=True, add_data_structures=True):

        shapes_list = []

        # Add the obstacles
        for obstacle in self.world_map.obstacles:
            shapes_list.append(view.get_obstacle_view_dict(obstacle))

        # Add the robots
        for robot in self.robots:
            shapes_list.extend(view.get_robot_view_dict(robot))

        # Add the start and the goal points to the frame
        shapes_list.append(view.get_goal_view_dict(self.world_map.goal))

        # Add data structures
        if add_data_structures:
            for controller in self.controllers:
                shapes_list.extend(view.get_data_structures_view_dict(controller.search_algorithm.draw_list))

        # Add the path
        if add_path:
            for robot, controller in zip(self.robots, self.controllers):
                path = controller.search_algorithm.path
                if len(path) > 0:
                    shapes_list.extend(view.get_path_view_dict([robot.current_pose] + path))

        return json.dumps(shapes_list)
