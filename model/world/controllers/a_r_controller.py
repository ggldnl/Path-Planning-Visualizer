import heapq

from model.geometry.segment import Segment
from model.geometry.point import Point

from model.world.controllers.controller import Controller


class Node:

    def __init__(self, point, score):
        self.point = point
        self.score = score

    def __str__(self):
        return f'Node({self.point}, score={self.score})'

    def __lt__(self, other):
        return self.score < other.score


class AStarController(Controller):

    def __init__(self, robot, map, step_size=1):

        super().__init__(robot, map)

        self.step_size = step_size

        self.start = None

        self.open_set = []  # Nodes to be explored
        self.closed_set = set()  # Explored nodes

        # Dictionary containing {node: parent_node}
        self.came_from = {}

        # Cost of getting from the start node to a given node: {point, cost}
        self.g_score = {}

        # Total cost of getting from the start node to the goal node through a given node
        self.f_score = {}

        self.initialize()

    def initialize(self):

        self.start = self.robot.current_pose.as_point()

        self.open_set = []
        self.closed_set = set()

        self.came_from = {}

        self.g_score[self.start] = 0.0

        self.f_score[self.start] = self.heuristic(self.start, self.map.goal)

        # Push f_score and Point
        heapq.heappush(self.open_set, Node(self.start, self.f_score[self.start]))

    def heuristic(self, node1, node2):
        # Euclidean distance as heuristic
        # return math.hypot(node2[0] - node1[0], node2[1] - node1[1])
        return node1.distance(node2)  # node1 and node2 are points and thus a built-in distance function

    def reconstruct_path(self, current):

        # Reversely add the parent node to the path
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)

        # Exclude the current point from the array; this way the array only contains points the robot should reach
        self.path = path[:-1]

        # Reset data structures
        self.open_set = []
        self.closed_set = set()  # Contains POINTS of visited nodes (check against map.get_neighbors)
        self.came_from = {}
        self.g_score = {self.start: 0.0}  # cost of getting from the start node to a given node
        self.f_score = {self.start: self.heuristic(self.start, self.map.goal)}

    def search(self):

        while self.open_set:

            # print([str(node.point) for node in self.open_set])

            node = heapq.heappop(self.open_set)
            current = node.point

            if current == self.map.goal:
                self.reconstruct_path(current)
                return

            self.closed_set.add(current)

            for neighbor in self.map.get_neighbors(current, step_size=self.step_size):

                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score >= self.g_score.get(neighbor, float('inf')):
                    continue

                self.came_from[neighbor] = current
                self.g_score[neighbor] = tentative_g_score
                self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.map.goal)

                if neighbor not in self.open_set:
                    heapq.heappush(self.open_set, Node(neighbor, self.f_score[neighbor]))

    def reset(self):
        self.path = []
        self.initialize()

    def get_draw_list(self):
        # For A* the segments we need to draw are the segments in self.path
        return ([Segment(self.path[i - 1], self.path[i]) for i in range(1, len(self.path))] +
                [Segment(Point(self.robot.current_pose.x, self.robot.current_pose.y), Point(self.robot.target_pose.x, self.robot.target_pose.y))])
