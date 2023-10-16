import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

NUM_CIRCLE_POINTS = 8


class Sphere:

    def __init__(self, radius, center):
        self.radius = radius
        self.center = center

    def get_projection(self, angles, offsets):
        return np.array(
            [
                [self.radius * np.cos(angle), self.radius * np.sin(angle)]
                for angle in np.linspace(0, 2 * np.pi, NUM_CIRCLE_POINTS)
            ]  # + [self.radius, 0]
        ) + offsets[:2]


class Box:

    def __init__(self, width, length, depth):
        self.width = width
        self.length = length
        self.depth = depth

    def get_projection(self, angles, offsets):
        # Define the points
        points = np.array([
            [self.length / 2, self.width / 2, -self.depth / 2],
            [-self.length / 2, self.width / 2, -self.depth / 2],
            [-self.length / 2, -self.width / 2, -self.depth / 2],
            [self.length / 2, -self.width / 2, -self.depth / 2],
            [self.length / 2, self.width / 2, self.depth / 2],
            [-self.length / 2, self.width / 2, self.depth / 2],
            [-self.length / 2, -self.width / 2, self.depth / 2],
            [self.length / 2, -self.width / 2, self.depth / 2]
        ])

        # Define the rotation angles (in radians) along each axis
        roll, pitch, yaw = angles

        # Compute the rotation matrix using Euler angles
        rotation_matrix = np.array([
            [np.cos(pitch) * np.cos(yaw), -np.cos(roll) * np.sin(yaw) + np.sin(roll) * np.sin(pitch) * np.cos(yaw),
             np.sin(roll) * np.sin(yaw) + np.cos(roll) * np.sin(pitch) * np.cos(yaw)],
            [np.cos(pitch) * np.sin(yaw), np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(pitch) * np.sin(yaw),
             -np.sin(roll) * np.cos(yaw) + np.cos(roll) * np.sin(pitch) * np.sin(yaw)],
            [-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)]
        ])

        # Rotate the points using the rotation matrix
        rotated_points = np.dot(points, rotation_matrix)

        # Project onto the XY plane
        projected_points = rotated_points[:, :2][:4]

        # Take only the outermost among them
        hull = ConvexHull(projected_points)
        outermost_points = [projected_points[i] for i in hull.vertices]

        # Assign an angle to each of them (accounting for the center of the polygon)
        points_with_angles = []
        for point in outermost_points:
            angle = np.arctan2(point[1], point[0])
            points_with_angles.append((point[0], point[1], angle))

        # Sort the points based on the angles
        points = np.array([[point[0], point[1]] for point in sorted(points_with_angles, key=lambda x: x[2])])

        # Return the ordered points with translation
        return points + np.array([offsets[0], offsets[1]])


class Cylinder:

    def __init__(self, radius, height):
        self.radius = radius
        self.height = height

    def get_projection(self, angles, offsets):
        # Define the points
        base = np.array(
            [
                [self.radius * np.cos(angle), self.radius * np.sin(angle), 0]
                for angle in np.linspace(0, 2 * np.pi, NUM_CIRCLE_POINTS)
            ]
        )
        points = np.concatenate((base, base))
        points[:, :, 4:] = self.height

        # Define the rotation angles (in radians) along each axis
        roll, pitch, yaw = angles

        # Compute the rotation matrix using Euler angles
        rotation_matrix = np.array([
            [np.cos(pitch) * np.cos(yaw), -np.cos(roll) * np.sin(yaw) + np.sin(roll) * np.sin(pitch) * np.cos(yaw),
             np.sin(roll) * np.sin(yaw) + np.cos(roll) * np.sin(pitch) * np.cos(yaw)],
            [np.cos(pitch) * np.sin(yaw), np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(pitch) * np.sin(yaw),
             -np.sin(roll) * np.cos(yaw) + np.cos(roll) * np.sin(pitch) * np.sin(yaw)],
            [-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)]
        ])

        # Rotate the points using the rotation matrix
        rotated_points = np.dot(points, rotation_matrix)

        # Project onto the XY plane
        projected_points = rotated_points[:, :2][:4]

        # Take only the outermost among them
        hull = ConvexHull(projected_points)
        outermost_points = [projected_points[i] for i in hull.vertices]

        # Assign an angle to each of them (accounting for the center of the polygon)
        points_with_angles = []
        for point in outermost_points:
            angle = np.arctan2(point[1], point[0])
            points_with_angles.append((point[0], point[1], angle))

        # Sort the points based on the angles
        points = np.array([[point[0], point[1]] for point in sorted(points_with_angles, key=lambda x: x[2])])

        # Return the ordered points with translation
        return points + np.array([offsets[0], offsets[1]])


b = Box(1, 1, 1)
points_1 = b.get_projection((0, 0, 0), (5, 0, 0))
x_1 = [point[0] for point in points_1]
y_1 = [point[1] for point in points_1]

points_2 = b.get_projection((0, 0, 0), (10, 10, 0))
x_2 = [point[0] for point in points_2]
y_2 = [point[1] for point in points_2]

# Plot the projected XY points
plt.figure()
plt.plot(x_1, y_1, color='red')
plt.plot(x_2, y_2, color='blue')
plt.title("Projected XY Points of Rotated 3D Box")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis('equal')  # Ensure aspect ratio is equal
plt.show()
