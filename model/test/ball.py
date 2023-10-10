from random import randint


class Ball:

    def __init__(self,
                 radius,
                 bounding_box_start_x,
                 bounding_box_end_x,
                 bounding_box_start_y,
                 bounding_box_end_y,
                 initial_x=None,
                 initial_y=None
                 ):

        # Bounding region in which the ball will move.
        # The ball does not have information on the frontend, so we need to constrain
        # the bounding box manually
        self.bounding_box_start_x = bounding_box_start_x
        self.bounding_box_end_x = bounding_box_end_x
        self.bounding_box_start_y = bounding_box_start_y
        self.bounding_box_end_y = bounding_box_end_y

        # Ball radius
        self.radius = radius

        # Coordinates of the ball
        if initial_x is not None and bounding_box_start_x <= initial_x <= bounding_box_end_x:
            self.x = initial_x
        else:
            self.x = randint(bounding_box_start_x, bounding_box_end_x)

        if initial_y is not None and bounding_box_start_y <= initial_y <= bounding_box_end_y:
            self.y = initial_y
        else:
            self.y = randint(bounding_box_start_y, bounding_box_end_y)

        # Initial speed
        self.vx = randint(50, 100)
        self.vy = randint(50, 100)

    def step(self, dt):

        if self.x - self.radius < self.bounding_box_start_x or self.x + self.radius >= self.bounding_box_end_x:
            self.vx *= -1

        if self.y - self.radius < self.bounding_box_start_y or self.y + self.radius >= self.bounding_box_end_y:
            self.vy *= -1

        self.x += self.vx * dt
        self.y += self.vy * dt
