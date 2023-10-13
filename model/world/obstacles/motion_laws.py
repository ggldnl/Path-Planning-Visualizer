import math


def no_motion(pose, vel, dt):
    return pose[0], pose[1], pose[2]

def simple_translation_motion(pose, vel, dt):
    """
    Describe the simplest motion: a plain translation using a velocity vector with no rotational component
    """
    return pose[0] + vel[0] * dt, pose[1] + vel[1] * dt, pose[2]


def translation_and_rotation_motion(pose, vel, dt):
    """
    Describe a translational motion with changing in the orientation
    """
    return pose[0] + vel[0] * dt, pose[1] + vel[1] * dt, (pose[2] + vel[2]) % 360 * dt


def spiral_motion(pose, vel, dt):
    """
    Describe a spiral motion. The vel vector is the following:
    (spiral_speed, angular_speed, rotational_speed)
    """

    # Calculate polar coordinates (r and theta)
    r = math.sqrt(pose[0] ** 2 + pose[1] ** 2)
    theta = math.atan2(pose[1], pose[0])

    # Update polar coordinates
    r += vel[0] * dt
    theta += vel[1] * dt

    # Convert polar coordinates back to Cartesian coordinates
    x = r * math.cos(theta)
    y = r * math.sin(theta)

    return x, y, pose[2]
