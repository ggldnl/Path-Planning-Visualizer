from model.geometry.polygon import Polygon
from model.geometry.circle import Circle
from model.geometry.rectangle import Rectangle
from model.geometry.segment import Segment


def polygon_intersects_circle(polygon, circle):
    for edge in polygon.get_edges():
        axis = edge.normal()
        min1, max1 = polygon.project(axis)
        min2, max2 = circle.project(axis)

        if min2 < min1 or max2 > max1:
            return False

    return True


def polygon_intersects_polygon(p1, p2):
    for edge in p1.get_edges() + p2.get_edges():
        axis = edge.normal()
        min1, max1 = p1.project(axis)
        min2, max2 = p2.project(axis)

        if max1 < min2 or max2 < min1:
            # If there is a gap along this axis, the polygons do not intersect
            return False

    return True


def polygon_intersects_segment(polygon, segment):
    for edge in polygon.get_edges() + [segment]:
        axis = edge.normal()
        min1, max1 = polygon.project(axis)
        min2, max2 = segment.project(axis)

        if max1 < min2 or max2 < min1:
            # If there is a gap along this axis, the polygons do not intersect
            return False

    return True


def segment_intersects_circle(segment, circle):
    # Check if either endpoint of the segment is inside the circle
    if circle.is_inside(segment.start):
        return True
    if circle.is_inside(segment.end):
        return True

    # Check for intersection between the segment and the circle
    closest_point = segment.closest_point(circle.get_center())
    distance_squared = (closest_point.x - circle.pose.x) ** 2 + (closest_point.y - circle.pose.y) ** 2
    return distance_squared <= circle.radius ** 2


def segment_intersects_polygon(segment, polygon):
    return polygon_intersects_segment(polygon, segment)


def segment_intersects_segment(s1, s2):
    axis1 = s1.normal()
    min1, max1 = s1.project(axis1)
    min2, max2 = s2.project(axis1)

    if max1 < min2 or max2 < min1:
        return False

    axis2 = s2.normal()
    min1, max1 = s1.project(axis2)
    min2, max2 = s2.project(axis2)

    if max1 < min2 or max2 < min1:
        return False

    return True


def circle_intersects_circle(c1, c2):
    # distance_between_centers = np.sqrt((c1.pose.x - c2.pose.x) ** 2 + (c1.pose.y - c2.pose.y) ** 2)
    distance_between_centers = c1.pose.distance(c2.pose)
    total_radii = c1.radius + c2.radius
    return distance_between_centers <= total_radii


def circle_intersects_polygon(circle, polygon):
    return polygon_intersects_circle(polygon, circle)


def circle_intersects_segment(circle, segment):
    return segment_intersects_circle(segment, circle)


def check_intersection(obj_1, obj_2):
    if isinstance(obj_1, Circle):
        if isinstance(obj_2, Circle):
            return circle_intersects_circle(obj_1, obj_2)
        elif isinstance(obj_2, Polygon) or isinstance(obj_2, Rectangle):
            return circle_intersects_polygon(obj_1, obj_2)
        elif isinstance(obj_2, Segment):
            return circle_intersects_segment(obj_1, obj_2)
        else:
            raise ValueError(f'Unsupported geometry type: {type(obj_2)}')
    elif isinstance(obj_1, Polygon) or isinstance(obj_2, Rectangle):
        if isinstance(obj_2, Circle):
            return polygon_intersects_circle(obj_1, obj_2)
        elif isinstance(obj_2, Polygon) or isinstance(obj_2, Rectangle):
            return polygon_intersects_polygon(obj_1, obj_2)
        elif isinstance(obj_2, Segment):
            return polygon_intersects_segment(obj_1, obj_2)
        else:
            raise ValueError(f'Unsupported geometry type: {type(obj_2)}')
    elif isinstance(obj_1, Segment):
        if isinstance(obj_2, Circle):
            return segment_intersects_circle(obj_1, obj_2)
        elif isinstance(obj_2, Polygon) or isinstance(obj_2, Rectangle):
            return segment_intersects_polygon(obj_1, obj_2)
        elif isinstance(obj_2, Segment):
            return segment_intersects_segment(obj_1, obj_2)
        else:
            raise ValueError(f'Unsupported geometry type: {type(obj_2)}')
    else:
        raise ValueError(f'Unsupported geometry type: {type(obj_1)}')
