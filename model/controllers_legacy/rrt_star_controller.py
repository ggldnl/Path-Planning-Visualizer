import numpy as np

from model.geometry.point import Point
from model.geometry.segment import Segment
from model.controllers_legacy.controller import Controller


class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0.0

    @property
    def x(self):
        return self.point.x

    @property
    def y(self):
        return self.point.y

    def distance(self, other_node):
        return self.point.distance(other_node.point)

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.point.x == other.point.x and self.point.y == other.point.y

    def __str__(self):
        return f'Node({self.point})'

    def __repr__(self):
        return self.__str__()


class Edge(Segment):

    def __init__(self, node_parent, node_child):
        super().__init__(node_parent.point, node_child.point)
        self.node_parent = node_parent
        self.node_child = node_child


class RRTStarController(Controller):

    def __init__(self,
                 robot,
                 map,
                 goal_sample_rate=0.05,
                 step_len=0.2,
                 search_radius=0.5,
                 max_iterations=8000,
                 iterations=4,
                 discretization_step=0.2
                 ):

        #  We use the discretization step to build a buffer for each segment and check intersections
        super().__init__(robot, map, iterations, discretization_step)

        # Percentage with which we use the goal as new point
        self.goal_sample_rate = goal_sample_rate

        # Maximum number of iterations (time constraint)
        self.max_iterations = max_iterations

        self.search_radius = search_radius
        self.step_len = step_len

        self._init()

    def _init(self):

        self.vertex = [Node(self.robot.current_pose.as_point())]
        self.edges = []

        self.current_iteration = 0

        # Disable moving obstacles until the path isn't found
        self.map.disable_moving_obstacles()

    def _search(self):

        if self.current_iteration < self.max_iterations:

            self.current_iteration += 1

            node_rand = self.generate_random_node()
            node_near = self.nearest_neighbor(node_rand)
            node_new = self.new_state(node_near, node_rand)

            # Skip if the generated node is exactly equal to the goal; This is because
            # we will get an exception when testing for collisions since a segment between
            # the goal and the node has length 0 (buffer generation in Polygon class).
            # The segment is used in self.search_goal_parent()

            if node_new and (node_new.point == self.map.goal or not self.check_collision(node_near.point, node_new.point)):

                neighbor_index = self.find_neighborhood(node_new)

                self.vertex.append(node_new)

                if neighbor_index:
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

                # Add the branches to the draw_list
                # self.draw_list.append(Segment(node_new.point, node_new.parent.point))
                self.update_draw_list()

        # elif not self.has_path() and not self.is_robot_at_goal() :
        elif not self.has_path() and not self.is_robot_at_goal():

            index = self.search_goal_parent()
            if index > 0:
                self.extract_path(self.vertex[index])

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.compute_cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def update_draw_list(self):
        self.draw_list = []
        for node in self.vertex:
            self.draw_list.append(node.point)
            if node.parent is not None:
                self.draw_list.append(Segment(node.parent.point, node.point))

    @staticmethod
    def compute_cost(node):
        n = node
        cost = 0.0
        while n.parent:
            cost += n.distance(n.parent)
            n = n.parent
        return cost

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        return self.compute_cost(node_start) + dist

    def find_neighborhood(self, node_new):
        n = len(self.vertex) + 1

        # r = min(self.search_radius * np.sqrt((np.log(n) / n)), self.discretization_step)
        r = self.search_radius

        dist_table = [np.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not node_new.point == self.map.goal and
                            not self.check_collision(node_new.point, self.vertex[ind].point)]

        return dist_table_index

    def search_goal_parent(self):
        dist_list = [self.distance_to_goal(n) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.search_radius]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.compute_cost(self.vertex[i]) for i in node_index
                         if not self.vertex[i].point == self.map.goal and
                         not self.check_collision(self.vertex[i].point, self.map.goal)]
            return node_index[int(np.argmin(cost_list))]

        # return len(self.vertex) - 1
        return -1

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            # 95% (by default) of the time, select a random point in space
            x = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist
            y = np.random.uniform(-2 * self.map.obs_max_dist, 0) + self.map.obs_max_dist

        else:
            # 5% (by default) of the time, select the goal
            x, y = self.map.goal

        return Node(Point(x, y))

    def nearest_neighbor(self, n):
        return min(self.vertex, key=lambda nd: nd.distance(n))

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        new_x = node_start.x + dist * np.cos(theta)
        new_y = node_start.y + dist * np.sin(theta)

        # new_y = round(new_y / self.step_len) * self.step_len
        # new_x = round(new_x / self.step_len) * self.step_len

        node_new = Node(Point(new_x, new_y))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        self.path = [self.map.goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            self.path.append(Point(node_now.x, node_now.y))

        # Remove first and last element, reverse the path
        self.path = self.path[::-1]

    def distance_to_goal(self, node):
        return node.point.distance(self.map.goal)

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return node_start.distance(node_end), np.arctan2(dy, dx)
