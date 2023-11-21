import heapq
import math

from model.geometry.point import Point
from model.world.robot.robot import Robot


class AStarController:
    def __init__(self, map, robot: Robot):
        self.path = []
        self.start = robot.current_pose.as_tuple()  # todo check if needed
        self.step_size = 0.1

        self.open_set = []  # nodes to be explored.
        self.closed_set = set()  # explored nodes
        self.came_from = {}
        self.g_score = {self.start: 0}  # cost of getting from the start node to a given node
        self.map = map
        self.f_score = {self.start: self.heuristic(self.start,
                                                   self.map.goal)}  # total cost of getting from the start node to the goal node through a given node

        heapq.heappush(self.open_set, (self.f_score[self.start], self.start))

    def heuristic(self, node1, node2):
        # Euclidean distance as heuristic
        return math.hypot(node2[0] - node1[0], node2[1] - node1[1])

    def reconstruct_path(self, current):
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        self.path = path  # [::-1]
        self.open_set = []  # nodes to be explored.
        self.closed_set = set()  # explored nodes
        self.came_from = {}
        self.g_score = {self.start: 0}  # cost of getting from the start node to a given node
        self.f_score = {self.start: self.heuristic(self.start, self.map.goal)}
        return self.path if isinstance(self.path, list) else list(self.path)

    def search(self):
        while self.open_set:
            current_f_score, current = heapq.heappop(self.open_set)

            if current == tuple(self.map.goal):
                print('Eureka!')
                return self.reconstruct_path(current)

            self.closed_set.add(current)

            for neighbor in self.map.get_neighbors(node=current,
                                                   step_size=self.step_size):
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score >= self.g_score.get(neighbor, float('inf')):
                    continue

                if self.is_collision(current, neighbor):
                    continue

                self.came_from[neighbor] = current
                self.g_score[neighbor] = tentative_g_score
                self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.map.goal)

                if neighbor not in [item[1] for item in self.open_set]:
                    heapq.heappush(self.open_set, (self.f_score[neighbor], neighbor))

        return None  # Path not found

    def is_collision(self, node1, node2):
        return self.map.check_collision(node1, node2)
