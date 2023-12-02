from model.controllers.search_algorithm import SearchAlgorithm

from model.geometry.point import Point


class DummySearch(SearchAlgorithm):

    def __init__(self, map, start=Point(0, 0), boundary=0.2, iterations=1, fake_search_steps=10):

        # Fake path, always the same
        self.fake_path = []

        # How many iterations to perform before "finding" the solution and (populating the path in this case)
        self.fake_search_steps = fake_search_steps
        self.current_fake_iteration = 0

        super().__init__(map, start, boundary, iterations=iterations)

    def pre_search(self):

        self.fake_path = [

            # Describe a square and then go to the goal
            Point(0, 0),
            Point(0, 1),
            Point(1, 1),
            Point(1, 0),
            Point(0, 0),

            # Go to the goal
            self.map.goal,
        ]

        self.current_fake_iteration = 0

    def step_search(self):
        self.current_fake_iteration += 1

    def post_search(self):
        self.path = self.fake_path.copy()

    def can_run(self):
        return self.current_fake_iteration < self.fake_search_steps
