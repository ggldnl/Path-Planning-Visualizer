import numpy as np


def no_motion(pose, vel, dt, **kwargs):
    return pose[0], pose[1], pose[2]


def simple_translation_motion(pose, vel, dt, **kwargs):
    """
    Describe the simplest motion: a plain translation using a velocity vector with no rotational component
    """
    return pose[0] + vel[0] * dt, pose[1] + vel[1] * dt, pose[2]


def translation_and_rotation_motion(pose, vel, dt, **kwargs):
    """
    Describe a translational motion with changing in the orientation
    """
    return pose[0] + vel[0] * dt, pose[1] + vel[1] * dt, (pose[2] + vel[2] * dt) % 360


def bouncing_back_motion(pose, vel, dt, **kwargs):

    if 'window_width' not in kwargs:
        raise ValueError('The width of the bounding box must be provided')
    window_width = kwargs['window_width']

    if 'window_height' not in kwargs:
        raise ValueError('The height of the bounding box must be provided')
    window_height = kwargs['window_height']

    x, y, theta = pose
    vx, vy, va = vel

    return x, y, theta


def circular_motion(pose, vel, dt, **kwargs):

    if 'center' not in kwargs:
        raise ValueError('The center of the circular path the obstacles needs to follow must be provided')
    center = kwargs['center']

    if 'radius' not in kwargs:
        raise ValueError('The radius of the circular path the obstacles needs to follow must be provided')
    radius = kwargs['radius']

    # Compute the initial angle
    x0, y0, theta = pose
    initial_angle = np.arctan2(y0, x0)

    # Compute the change in angle
    vx, vy, va = vel
    angular_displacement = (np.sqrt(vx ** 2 + vy ** 2) * dt) / radius

    # Calculate the new angle
    new_angle = initial_angle + angular_displacement

    # Convert the new angle back to Cartesian coordinates
    x1 = center.x + radius * np.cos(new_angle)
    y1 = center.y + radius * np.sin(new_angle)

    return x1, y1, pose[2]
