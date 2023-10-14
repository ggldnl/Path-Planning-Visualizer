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


def bounded_window_motion(pose, vel, dt, width=50, height=50):

    x = pose[0]
    y = pose[1]
    theta = pose[2]

    vx = vel[0]
    vy = vel[1]

    if x <= -width / 2 or x >= width / 2:
        print('inverted speed on x')
        vx *= -1

    if y <= -height / 2 or y >= height / 2:
        print('inverted speed on x')
        vy *= -1

    return x + vx * dt, y + vy * dt, theta
