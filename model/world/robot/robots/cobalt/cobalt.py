from model.world.robot.differential_drive_robot import DifferentialDriveRobot
from model.geometry.polygon import Polygon

import numpy as np

from model.world.sensors.proximity_sensor import ProximitySensor


class Cobalt(DifferentialDriveRobot):

    def __init__(self):

        # Build the base

        radius = 0.05  # 5 cm
        base_upper_half = [
            [radius * np.cos(angle), radius * np.sin(angle)]
            for angle in np.linspace(np.deg2rad(25), np.deg2rad(155), 8)
        ]

        base_lower_half = [
            [radius * np.cos(angle), radius * np.sin(angle)]
            for angle in np.linspace(np.deg2rad(205), np.deg2rad(335), 8)
        ]

        """
        left_wheel_housing = [
            [-0.034, base_upper_half[-1][1]],
            [-0.034, base_lower_half[0][1]]
        ]

        right_wheel_housing = [
            [0.034, base_lower_half[-1][1]],
            [0.034, base_upper_half[0][1]]
        ]
        """

        base_polygon = Polygon(
            base_upper_half +   # left_wheel_housing +
            base_lower_half     # + right_wheel_housing
        )

        # Wheels have radius 20 mm and height 10 mm
        wheel_radius = 0.02
        wheel_height = 0.01
        wheel_polygon = Polygon([
            [wheel_height / 2, wheel_radius],
            [-wheel_height / 2, wheel_radius],
            [-wheel_height / 2, -wheel_radius],
            [wheel_height / 2, -wheel_radius]
        ])

        left_wheel_pose = (0.04, 0, 0)
        right_wheel_pose = (-0.04, 0, 0)

        left_wheel_polygon = wheel_polygon.copy()
        left_wheel_polygon.transform(left_wheel_pose)

        right_wheel_polygon = wheel_polygon.copy()
        right_wheel_polygon.transform(right_wheel_pose)

        caster_wheel_radius = 0.005
        caster_wheel = Polygon([
            [caster_wheel_radius * np.cos(angle), caster_wheel_radius * np.sin(angle)]
            for angle in np.linspace(0, 2 * np.pi, 8)
        ])

        caster_wheel_pose = (0, -0.035, 0)

        caster_wheel.transform(caster_wheel_pose)

        bodies = [
            base_polygon,
            left_wheel_polygon,
            right_wheel_polygon,
            caster_wheel
        ]

        super().__init__(bodies)

        # Add the sensors
        left_sensor = ProximitySensor('left_sensor', 0.04, 30, is_deg=True)
        front_sensor = ProximitySensor('front_sensor', 0.04, 30, is_deg=True)
        right_sensor = ProximitySensor('right_sensor', 0.04, 30, is_deg=True)

        # Left and right sensors are at 45 and 135 deg
        a = radius * np.cos(np.pi / 4) - 0.005  # Subtract half a centimeter to make the sensor fit inside the robot
        b = radius * np.sin(np.pi / 4) - 0.005

        self.add_sensor(left_sensor, (a, b, -np.pi / 4), is_deg=False)
        self.add_sensor(front_sensor, (0, radius - 0.005, 0), is_deg=False)
        self.add_sensor(right_sensor, (-a, b, np.pi / 4), is_deg=False)

        # Robot is headed on positive x values
        for body in bodies:
            body.rotate_around(0, 0, -90)


if __name__ == '__main__':

    robot = Cobalt()

    import matplotlib.pyplot as plt

    # Plot the projected XY points
    plt.figure()

    for polygon in robot.bodies:
        x = [point.x for point in polygon.points]
        y = [point.y for point in polygon.points]
        plt.plot(x, y, color='blue')

    for sensor in robot.sensors:
        x = [point.x for point in sensor.polygon.points]
        y = [point.y for point in sensor.polygon.points]
        plt.plot(x, y, color='green')

    plt.title("Projected XY Points")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')  # Ensure aspect ratio is equal
    plt.show()

