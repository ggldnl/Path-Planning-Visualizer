import model.geometry.linalg as linalg


def check_nearness(polygon1, polygon2):
    """
    Fast test to determine if two geometries might be touching. It can be used
    to exclude from the computation objects that are far away and that for sure
    do not intersect.
    """

    c1, r1 = polygon1.bounding_circle
    c2, r2 = polygon2.bounding_circle
    return c1.distance(c2) <= r1 + r2


def convex_polygon_intersect_test(polygon1, polygon2):
    """
    Determine if two convex polygons intersect
    """

    # Find the polygon that has fewer sides so that we can do fewer checks
    polygon_a = polygon1 if len(polygon1) <= len(polygon2) else polygon2
    polygon_b = polygon2 if len(polygon1) > len(polygon2) else polygon1

    # Perform Separating Axis Test
    intersect = True
    edge_index = 0
    edges = polygon_a.get_edges() + polygon_b.get_edges()

    # Loop through the edges of polygonA searching for a separating axis
    while intersect and edge_index < len(edges):

        # Get an axis normal to the current edge
        edge = edges[edge_index]
        edge_vector = linalg.sub(edge[1], edge[0])
        projection_axis = linalg.lnormal(edge_vector)

        # Get the projection ranges for each polygon onto the projection axis
        min_a, max_a = range_project_polygon(projection_axis, polygon_a)
        min_b, max_b = range_project_polygon(projection_axis, polygon_b)

        # test if projections overlap
        if min_a > max_b or max_a < min_b:
            intersect = False
        edge_index += 1

    return intersect


def range_project_polygon(axis_vector, polygon):
    """
    Get the min and max dot-products of a projection axis and the vertices of a polygon -
    this is sufficient for overlap comparison
    """

    vertices = polygon.points

    c = linalg.dot(axis_vector, vertices[0])
    min_c = c
    max_c = c

    for i in range(1, len(vertices)):

        # Compute the dot product between the axis vector and the vertex
        c = linalg.dot(axis_vector, vertices[i])

        # Check if the result is min/max
        if c < min_c:
            min_c = c
        elif c > max_c:
            max_c = c

    return min_c, max_c
