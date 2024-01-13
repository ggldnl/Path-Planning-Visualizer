from model.world.robot.differential_drive_robot import DifferentialDriveRobot
from model.geometry.polygon import Polygon
from model.geometry.circle import Circle
from model.geometry.pose import Pose

import numpy as np


class Cobalt(DifferentialDriveRobot):

    def __init__(self):

        # Build the base

        radius = 0.05  # 5 cm

        """
        base_upper_half = [
            [radius * np.cos(angle), radius * np.sin(angle)]
            for angle in np.linspace(np.deg2rad(25), np.deg2rad(155), 8)
        ]

        base_lower_half = [
            [radius * np.cos(angle), radius * np.sin(angle)]
            for angle in np.linspace(np.deg2rad(205), np.deg2rad(335), 8)
        ]

        left_wheel_housing = [
            [-0.034, base_upper_half[-1][1]],
            [-0.034, base_lower_half[0][1]]
        ]

        right_wheel_housing = [
            [0.034, base_lower_half[-1][1]],
            [0.034, base_upper_half[0][1]]
        ]
        
        base_polygon = Polygon(
            base_upper_half +   # left_wheel_housing +
            base_lower_half     # + right_wheel_housing
        )
        """

        base_polygon = Circle(0, 0, radius)

        # Wheels have radius 20 mm and height 10 mm
        wheel_radius = 0.02
        wheel_height = 0.01
        wheel_polygon = Polygon([
            [wheel_height / 2, wheel_radius],
            [-wheel_height / 2, wheel_radius],
            [-wheel_height / 2, -wheel_radius],
            [wheel_height / 2, -wheel_radius]
        ])

        # left_wheel_pose = Pose(0.04, 0, 0)
        # right_wheel_pose = Pose(-0.04, 0, 0)

        left_wheel_polygon = wheel_polygon.copy()
        left_wheel_polygon.transform(0.04, 0, 0)

        right_wheel_polygon = wheel_polygon.copy()
        right_wheel_polygon.transform(-0.04, 0, 0)

        caster_wheel_radius = 0.005
        """
        caster_wheel = Polygon([
            [caster_wheel_radius * np.cos(angle), caster_wheel_radius * np.sin(angle)]
            for angle in np.linspace(0, 2 * np.pi, 8)
        ])
        """
        caster_wheel = Circle(0, 0, caster_wheel_radius)

        # caster_wheel_pose = Pose(0, -0.035, 0)

        caster_wheel.transform(0, -0.035, 0)

        bodies = [
            base_polygon,
            left_wheel_polygon,
            right_wheel_polygon,
            caster_wheel
        ]

        # Define the wheelbase
        wheelbase = 0.035 * 2

        super().__init__(bodies, wheelbase)

        # Robot is headed on positive x values
        orientation = 3/2 * np.pi
        for body in bodies:
            body.rotate_around(0, 0, orientation)


if __name__ == '__main__':

    robot = Cobalt()

    import matplotlib.pyplot as plt

    # Plot the projected XY points
    plt.figure()

    for polygon in robot.bodies:
        if isinstance(polygon, Polygon):
            x = [point.x for point in polygon.points] + [polygon.points[0].x]
            y = [point.y for point in polygon.points] + [polygon.points[0].y]
        else:
            theta = np.linspace(0, 2 * np.pi, 100)
            x = polygon.pose.x + polygon.radius * np.cos(theta)
            y = polygon.pose.y + polygon.radius * np.sin(theta)
        plt.plot(x, y, color='blue')

    plt.title("Projected XY Points")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')  # Ensure aspect ratio is equal
    plt.show()

