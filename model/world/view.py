from model.geometry.polygon import Polygon
from model.geometry.segment import Segment
from model.geometry.circle import Circle
from model.geometry.point import Point

from model.world.color_palette import *


def _polygon_dict(points, fill_color, border_color, line_width):
    return {
        "type": "polygon",
        "points": points,
        "fill_color": fill_color,
        "border_color": border_color,
        "line_width": line_width,
    }


def _circle_dict(center, radius, fill_color, border_color, line_width):
    return {
        "type": "circle",
        "center": [center.x, center.y],
        "radius": radius,
        "fill_color": fill_color,
        "border_color": border_color,
        "line_width": line_width
    }


def _segment_dict(p1, p2, color, line_width):
    return {
        "type": "segment",
        "p1": [p1.x, p1.y],
        "p2": [p2.x, p2.y],
        "color": color,
        "line_width": line_width
    }


def __ellipse_dict(ellipse, fill_color, border_color, line_width):
    return {
        "type": "ellipse",
        "center": [ellipse.pose.x, ellipse.pose.y],
        "a": ellipse.a,
        "b": ellipse.b,
        "phi": ellipse.phi,
        "fill_color": fill_color,
        "border_color": border_color,
        "line_width": line_width
    }


def get_obstacle_view_dict(obstacle):
    return _polygon_dict(obstacle.polygon.to_point_array(), obstacle_fill_color, obstacle_border_color, 1.0)


def get_robot_view_dict(robot):
    shapes_list = []
    for body in robot.bodies:
        if isinstance(body, Polygon):
            shapes_list.append(_polygon_dict(body.to_point_array(), robot_fill_color, robot_border_color, 1.0))
        elif isinstance(body, Circle):
            shapes_list.append(_circle_dict(body.pose, body.radius, robot_fill_color, robot_border_color, 1.0))
    return shapes_list


def get_goal_view_dict(goal):
    return _circle_dict(goal, 0.025, goal_fill_color, goal_border_color, 1.0)


def get_data_structures_view_dict(draw_list):
    shapes_list = []
    for structure in draw_list:
        if isinstance(structure, Polygon):
            shapes_list.append(
                _polygon_dict(structure.to_point_array(), tile_color, 'transparent', 0.5)
            )
        elif isinstance(structure, Segment):
            shapes_list.append(
                _segment_dict(structure.start, structure.end, path_color, 0.5)
            )
        elif isinstance(structure, Point):
            shapes_list.append(
                _circle_dict(structure, 0.025, path_color, 'transparent', 0.5)
            )
    return shapes_list


def get_path_view_dict(path):
    shapes_list = []
    for i in range(1, len(path)):
        shapes_list.append(
            _segment_dict(path[i-1], path[i], path_color, 1.0)
        )
    return shapes_list


def get_ellipse_view_dict(ellipse):
    return __ellipse_dict(ellipse, 'transparent', ellipse_color, 0.5)

