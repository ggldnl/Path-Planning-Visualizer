from model.geometry.polygon import Polygon
from model.geometry.point import Point
import numpy as np
import random


class Obstacle:

    def __init__(self, polygon, vel=(0, 0, 0)):

        self.polygon = polygon

        self.vel = vel

        # Global speed multipliers (the obstacles moves proportionally to these values)
        self.linear_speed_multiplier = 1
        self.angular_speed_multiplier = 1

    def set_random_velocity_vector(self):
        self.vel = (random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5))

    def step_motion(self, dt):
        """
        Simulate the obstacle's motion over the given time interval
        """

        # Update the pose based on the motion law (translation + rotation)
        x, y, z = self.polygon.pose
        vx, vy, vz = self.vel
        lsm = self.linear_speed_multiplier
        asm = self.angular_speed_multiplier

        new_x = x + vx * lsm * dt
        new_y = y + vy * lsm * dt
        new_z = (z + vz * asm * dt) % (2 * np.pi)

        # self.polygon.translate_to(self.pose[0], self.pose[1])
        self.polygon.transform_to(new_x, new_y, new_z)

    def copy(self):
        new_polygon = self.polygon.copy()
        new_vel = (self.vel[0], self.vel[1], self.vel[2])
        return Obstacle(new_polygon, new_vel)

    def to_dict(self):
        return {'polygon': self.polygon.to_dict(), 'vel': self.vel}

    @classmethod
    def from_dict(cls, dictionary):
        polygon = Polygon.from_dict(dictionary['polygon'])
        vel = tuple(dictionary['vel'])
        return Obstacle(polygon, vel)


class RectangularObstacle(Obstacle):

    def __init__(self, width, height, vel=None):

        if width <= 0:
            raise ValueError(f'Invalid width: {width}')

        if height <= 0:
            raise ValueError(f'Invalid height: {height}')

        polygon = Polygon([
            Point(0, 0),
            Point(0, height),
            Point(width, height),
            Point(width, 0)
        ])

        super().__init__(polygon, vel)


if __name__ == '__main__':

    import matplotlib.pyplot as plt

    # Plot the projected XY points
    plt.figure()

    # Define a random polygon
    polygon = Polygon.generate_random_polygon(4, 10, )

    # Create an obstacle
    obstacle = Obstacle(polygon, (1, 0, 0))  # linear speed 1 m/s on x
    obstacle_cpy = obstacle.copy()

    obstacle_cpy.step_motion(0.05)

    x_1 = [point.x for point in obstacle.polygon.points] + [obstacle.polygon.points[0].x]
    y_1 = [point.y for point in obstacle.polygon.points] + [obstacle.polygon.points[0].y]
    plt.plot(x_1, y_1, color='blue')

    x_2 = [point.x for point in obstacle_cpy.polygon.points] + [obstacle_cpy.polygon.points[0].x]
    y_2 = [point.y for point in obstacle_cpy.polygon.points] + [obstacle_cpy.polygon.points[0].y]
    plt.plot(x_2, y_2, color='red')

    plt.title("Projected XY Points")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')  # Ensure aspect ratio is equal
    plt.show()
