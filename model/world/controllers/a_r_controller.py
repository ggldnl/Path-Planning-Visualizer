import heapq
import math

from model.world.map.map import Map


class AStarController:
    def __init__(self, start, goal, map: Map):
        self.start = start
        self.goal = goal
        self.map = map

        self.open_set = []
        self.closed_set = set()
        self.came_from = {}
        self.g_score = {self.start: 0}
        self.f_score = {self.start: self.heuristic(self.start, self.goal)}

        heapq.heappush(self.open_set, (self.f_score[self.start], self.start))

    def heuristic(self, node1, node2):
        # Euclidean distance as heuristic
        return math.hypot(node2[0] - node1[0], node2[1] - node1[1])

    def reconstruct_path(self, current):
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        return path[::-1]

    def search(self):
        while self.open_set:
            current_f_score, current = heapq.heappop(self.open_set)

            if current == self.goal:
                return self.reconstruct_path(current)

            self.closed_set.add(current)

            for neighbor in self.map.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score >= self.g_score.get(neighbor, float('inf')):
                    continue

                if self.is_collision(current, neighbor):
                    continue

                self.came_from[neighbor] = current
                self.g_score[neighbor] = tentative_g_score
                self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)

                if neighbor not in [item[1] for item in self.open_set]:
                    heapq.heappush(self.open_set, (self.f_score[neighbor], neighbor))

        return None  # Path not found

    def is_collision(self, node1, node2):
        return self.map.is_obstacle([node1, node2])

    def update_map(self, new_map):
        self.map = new_map

    def update_goal(self, new_goal):
        self.goal = new_goal
        self.f_score[self.goal] = self.g_score[self.goal] + self.heuristic(self.goal, self.goal)

    def update_start(self, new_start):
        self.start = new_start
        self.g_score[self.start] = 0
        self.f_score[self.start] = self.heuristic(self.start, self.goal)

        heapq.heappush(self.open_set, (self.f_score[self.start], self.start))
