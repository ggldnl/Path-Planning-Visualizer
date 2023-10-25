import numpy as np
import heapq

from model.world.controllers.controller import Controller


def get_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).

    Args:
    x1 (float): x-coordinate of the first point.
    y1 (float): y-coordinate of the first point.
    x2 (float): x-coordinate of the second point.
    y2 (float): y-coordinate of the second point.

    Returns:
    float: Euclidean distance between the two points.
    """
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class DijkstraController(Controller):
    def __init__(self, map, robot):
        super(DijkstraController, self).__init__(robot)
        self.map = map
        self.goal = None
        self.path = []
        self.grid_size = 0.1  # Size of the grid in meters
        self.create_grid()

    def create_grid(self):
        min_x, min_y, max_x, max_y = self.map.get_bounds()
        self.xs = np.arange(min_x, max_x, self.grid_size)
        self.ys = np.arange(min_y, max_y, self.grid_size)
        self.grid = np.zeros((len(self.xs), len(self.ys)))

    def get_nearest_grid_point(self, x, y):
        x_idx = np.argmin(np.abs(self.xs - x))
        y_idx = np.argmin(np.abs(self.ys - y))
        return x_idx, y_idx

    def update_goal(self):
        self.goal = self.get_nearest_grid_point(self.map.goal.x, self.world.map.goal.y)
        self.calculate_path()

    def calculate_path(self):
        # Reset the grid
        self.grid.fill(np.inf)
        start_x, start_y = self.get_nearest_grid_point(self.robot.x, self.robot.y)
        self.grid[start_x, start_y] = 0

        # Priority queue for Dijkstra's algorithm
        pq = []
        heapq.heappush(pq, (0, (start_x, start_y)))

        # Dijkstra's algorithm
        while pq:
            cost, (x, y) = heapq.heappop(pq)
            if (x, y) == self.goal:
                break

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue

                    nx, ny = x + dx, y + dy
                    if 0 <= nx < len(self.xs) and 0 <= ny < len(self.ys):
                        new_cost = cost + get_distance(self.xs[x], self.ys[y], self.xs[nx], self.ys[ny])
                        if new_cost < self.grid[nx, ny]:
                            self.grid[nx, ny] = new_cost
                            heapq.heappush(pq, (new_cost, (nx, ny)))

        # Backtrack to get the path
        self.path = []
        x, y = self.goal
        while (x, y) != (start_x, start_y):
            self.path.append((self.xs[x], self.ys[y]))
            min_cost = np.inf
            next_x, next_y = None, None
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue

                    nx, ny = x + dx, y + dy
                    if 0 <= nx < len(self.xs) and 0 <= ny < len(self.ys) and self.grid[nx, ny] < min_cost:
                        min_cost = self.grid[nx, ny]
                        next_x, next_y = nx, ny

            x, y = next_x, next_y

    def get_control(self):
        if not self.path:
            return 0, 0

        goal_x, goal_y = self.path[-1]
        angle_to_goal = np.arctan2(goal_y - self.robot.y, goal_x - self.robot.x)
        angle_diff = angle_to_goal - self.robot.theta

        if abs(angle_diff) > np.pi:
            angle_diff -= 2 * np.pi * np.sign(angle_diff)

        if abs(angle_diff) > 0.1:
            return 0, np.sign(angle_diff) * 0.5
        else:
            return 0.1, 0
