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
    return pose[0] + vel[0] * dt, pose[1] + vel[1] * dt, (pose[2] + vel[2] * dt) % 360


def circular_motion(pose, vel, dt):
    return pose[0], pose[1], pose[2]