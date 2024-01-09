
class Rect:
    """
    Rectangle with lower left corner at (minx, miny) and upper right corner at (maxx, maxy)
    """

    def __init__(self, minx, miny, maxx, maxy):

        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy

        self.west_edge = minx
        self.east_edge = maxx
        self.north_edge = miny
        self.south_edge = maxy

    def __str__(self):
        return '({:.2f}, {:.2f}, {:.2f}, {:.2f})'.format(self.west_edge,
                                                         self.north_edge, self.east_edge, self.south_edge)

    def __repr__(self):
        return self.__str__()

    def contains(self, point):
        return self.west_edge <= point.x <= self.east_edge and self.north_edge <= point.y <= self.south_edge

    def intersects(self, other):
        """
        Check for intersection with another rectangle
        """

        return not (other.west_edge > self.east_edge or
                    other.east_edge < self.west_edge or
                    other.north_edge > self.south_edge or
                    other.south_edge < self.north_edge)


class QuadTree:

    def __init__(self, boundary, max_points=4, depth=0):

        self.boundary = boundary
        self.max_points = max_points
        self.points = []
        self.depth = depth

        # Whether this node has divided (branched) or not.
        self.divided = False

        # The boundaries of the four children nodes are "northwest",
        # "northeast", "southeast" and "southwest" quadrants within the
        # boundary of the current node.
        self.nw = None
        self.ne = None
        self.se = None
        self.sw = None

    def divide(self):
        """
        Branch this node by spawning four children nodes.
        """

        minx = self.boundary.minx
        maxx = self.boundary.maxx
        miny = self.boundary.miny
        maxy = self.boundary.maxy

        cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
        w, h = (maxx - minx) / 2, (maxy - miny) / 2

        # The boundaries of the four children nodes are "northwest",
        # "northeast", "southeast" and "southwest" quadrants within the
        # boundary of the current node.
        self.nw = QuadTree(Rect(cx, cy - h / 2, w / 2, h / 2),
                           self.max_points, self.depth + 1)
        self.ne = QuadTree(Rect(cx + w / 2, cy - h / 2, w / 2, h / 2),
                           self.max_points, self.depth + 1)
        self.se = QuadTree(Rect(cx + w / 2, cy + h / 2, w / 2, h / 2),
                           self.max_points, self.depth + 1)
        self.sw = QuadTree(Rect(cx - w / 2, cy + h / 2, w / 2, h / 2),
                           self.max_points, self.depth + 1)
        self.divided = True

    def insert(self, point, index=None):
        """
        Try to insert the point into the tree
        """

        # If index is provided, remove the point at that index
        if index is not None and 0 <= index < len(self.points):
            del self.points[index]

        if not self.boundary.contains(point):
            # The point does not lie inside the boundary: bail.
            return False
        if len(self.points) < self.max_points:
            # There's room for our point without dividing the QuadTree.
            self.points.append(point)
            return True

        # No room: divide if necessary, then try the sub-quads.
        if not self.divided:
            self.divide()

        return (self.ne.insert(point, index) or
                self.nw.insert(point, index) or
                self.se.insert(point, index) or
                self.sw.insert(point, index))

    def delete(self, index):
        """Delete the point at the given index."""
        if 0 <= index < len(self.points):
            del self.points[index]

    def query(self, boundary, found_points):
        """
        Find the points in the tree that lie within boundary
        """

        if not self.boundary.intersects(boundary):
            # If the domain of this node does not intersect the search
            # region, we don't need to look in it for points.
            return False

        # Search this node's points to see if they lie within boundary
        for point in self.points:
            if boundary.contains(point):
                found_points.append(point)

        # If this node has children, search them too.
        if self.divided:
            self.nw.query(boundary, found_points)
            self.ne.query(boundary, found_points)
            self.se.query(boundary, found_points)
            self.sw.query(boundary, found_points)

        return found_points

    def query_circle(self, boundary, centre, radius, found_points):
        """
        Find the points in the tree that lie within radius of centre.
        """

        if not self.boundary.intersects(boundary):
            # If the domain of this node does not intersect the search
            # region, we don't need to look in it for points.
            return False

        # Search this node's points to see if they lie within boundary
        # and also lie within a circle of given radius around the centre point.
        for point in self.points:
            if (boundary.contains(point) and
                    point.distance_to(centre) <= radius):
                found_points.append(point)

        # Recurse the search into this node's children.
        if self.divided:
            self.nw.query_circle(boundary, centre, radius, found_points)
            self.ne.query_circle(boundary, centre, radius, found_points)
            self.se.query_circle(boundary, centre, radius, found_points)
            self.sw.query_circle(boundary, centre, radius, found_points)

        return found_points

    def query_radius(self, centre, radius, found_points):
        """
        Find the points in the tree that lie within radius of centre.
        """

        # First find the square that bounds the search circle as a Rect object.
        boundary = Rect(*centre, 2*radius, 2*radius)
        return self.query_circle(boundary, centre, radius, found_points)

    def __len__(self):
        """
        Return the number of points in the quadtree.
        """

        npoints = len(self.points)
        if self.divided:
            npoints += len(self.nw)+len(self.ne)+len(self.se)+len(self.sw)
        return npoints
