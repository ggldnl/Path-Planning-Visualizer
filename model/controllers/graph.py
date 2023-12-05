class Node:

    def __init__(self, point, parent=None, cost=0, heuristic=0):
        self.point = point
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic

    def __str__(self):
        return f'Node ({self.point})'

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.point == other.point

    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost + other.heuristic

    def __hash__(self):
        # Use the hash value of the point attribute
        return hash(self.point)


class Edge:

    def __init__(self, parent_node, child_node):
        self.parent_node = parent_node
        self.child_node = child_node

    def __eq__(self, other):
        if not isinstance(other, Edge):
            return False
        return self.parent_node == other.parent_node and self.child_node == other.child_node
