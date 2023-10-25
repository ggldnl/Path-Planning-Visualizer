import heapq

from .controller import Controller
from ..robots.robot import Robot
import numpy as np
from ..map.map import Map


class Node:
    def __init__(self, pose, cost=float('inf')):
        self.pose = pose  # A tuple (x, y, theta)
        self.cost = cost  # Cost to reach this node
        self.edges = []  # List of tuples (neighbor_node, edge_cost)
        self.visited = False  # Flag to check if this node has been visited

    def add_edge(self, neighbor_node, edge_cost):
        self.edges.append((neighbor_node, edge_cost))


class DijkstraController(Controller):
    def step_motion(self, map: Map):
        goal_pos, robot_pose = self.get_current_state(map)
        path = self.dijkstra(robot_pose, goal_pos, map.obstacles)
        if path:
            next_point = path[1]
            self.drive_train = np.array(next_point[:2]) - np.array(robot_pose[:2])
        else:
            print("No path found")
            self.drive_train = np.array([0, 0])

    def dijkstra(self, start, goal, obstacles):
        nodes = {}
        start_node = Node(start, cost=0)
        goal_node = Node(goal)
        nodes[start] = start_node
        nodes[goal] = goal_node

        # Create nodes around each obstacle
        for obstacle in obstacles:
            obstacle_points = self.generate_points_around_obstacle(obstacle)
            for point in obstacle_points:
                if point not in nodes:
                    nodes[point] = Node(point)

        # Connect nodes, avoiding obstacles
        for point, node in nodes.items():
            for other_point, other_node in nodes.items():
                if point != other_point and not self.check_intersection_with_obstacles(point, other_point, obstacles):
                    distance = np.linalg.norm(np.array(point[:2]) - np.array(other_point[:2]))
                    node.add_edge(other_node, distance)

        # Dijkstra's algorithm
        priority_queue = [(0, start_node)]
        while priority_queue:
            current_cost, current_node = heapq.heappop(priority_queue)
            if current_node.visited:
                continue
            current_node.visited = True

            if current_node == goal_node:
                return self.reconstruct_path(start_node, goal_node)

            for neighbor_node, edge_cost in current_node.edges:
                if neighbor_node.visited:
                    continue
                new_cost = current_cost + edge_cost
                if new_cost < neighbor_node.cost:
                    neighbor_node.cost = new_cost
                    heapq.heappush(priority_queue, (new_cost, neighbor_node))

        return None

    def generate_points_around_obstacle(self, obstacle):
        points = []
        for edge in obstacle.polygon.edges:
            start, end = edge.start, edge.end

            # Add the start and end points of each edge
            points.append((start.x, start.y))
            points.append((end.x, end.y))

            # Optionally, add the midpoint of the edge
            mid_x = (start.x + end.x) / 2
            mid_y = (start.y + end.y) / 2
            points.append((mid_x, mid_y))

        # Removing duplicates, if any
        unique_points = set(points)
        return list(unique_points)

    def check_intersection_with_obstacles(self, point1, point2, obstacles):
        # Implement this method to check if the line between point1 and point2
        # intersects with any obstacles
        pass

    def reconstruct_path(self, start_node, goal_node):
        # Reconstruct the path from start to goal
        path = []
        current_node = goal_node
        while current_node != start_node:
            path.append(current_node.pose)
            current_node = current_node.parent  # Assumes that parent nodes are set during Dijkstra's algorithm
        path.append(start_node.pose)
        path.reverse()
        return path
