import heapq
import math

from model.world.map.map_rtree import Map
from model.world.robot.robot import Robot


class AStarController:
    def __init__(self, goal, robot: Robot):
        self.path = []
        self.start = robot.current_pose #todo check if needed
        self.step_size = 0.1

        self.open_set = [] #nodes to be explored.
        self.closed_set = set() #explored nodes
        self.came_from = {}
        self.g_score = {self.start: 0} #cost of getting from the start node to a given node
        self.f_score = {self.start: self.heuristic(self.start, goal)} #total cost of getting from the start node to the goal node through a given node

        heapq.heappush(self.open_set, (self.f_score[self.start], self.start))

    def heuristic(self, node1, node2):
        # Euclidean distance as heuristic
        return math.hypot(node2[0] - node1[0], node2[1] - node1[1])

    def reconstruct_path(self, map, current):
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        self.path = path#[::-1]
        self.open_set = []  # nodes to be explored.
        self.closed_set = set()  # explored nodes
        self.came_from = {}
        self.g_score = {self.start: 0}  # cost of getting from the start node to a given node
        self.f_score = {self.start: self.heuristic(self.start, map.goal)}
        return self.path if isinstance(self.path, list) else list(self.path)

    def search(self, map):
        while self.open_set:
            current_f_score, current = heapq.heappop(self.open_set)

            if current == tuple(map.goal):
                return self.reconstruct_path(map, current)

            self.closed_set.add(current)

            for neighbor in map.get_neighbors(node=current,
                                                   step_size=self.step_size):
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score >= self.g_score.get(neighbor, float('inf')):
                    continue

                if self.is_collision(map, current, neighbor):
                    continue

                self.came_from[neighbor] = current
                self.g_score[neighbor] = tentative_g_score
                self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, map.goal)

                if neighbor not in [item[1] for item in self.open_set]:
                    heapq.heappush(self.open_set, (self.f_score[neighbor], neighbor))

        return None  # Path not found

    def is_collision(self, map, node1, node2):
        return map.is_obstacle(node1, node2)

    '''
    def update_goal(self, new_goal):
        self.goal = new_goal
        self.f_score[self.goal] = self.g_score[self.goal] + self.heuristic(self.goal, self.goal)

    def update_start(self, new_start):
        self.start = new_start
        self.g_score[self.start] = 0
        self.f_score[self.start] = self.heuristic(self.start, self.goal)

        heapq.heappush(self.open_set, (self.f_score[self.start], self.start))
    '''
