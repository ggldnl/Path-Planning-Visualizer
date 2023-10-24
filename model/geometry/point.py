from numbers import Number
import math


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Point(x={self.x}, y={self.y})"

    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return self.x == other.x and self.y == other.y

    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        raise IndexError("Point index out of range")

    # Python 3.x uses __truediv__ and __floordiv__. __div__ is 2.x-only.

    def __itruediv__(self, other):
        if isinstance(other, Point):
            self.x = self.x / other.x
            self.x = self.y / other.y
            return self
        elif isinstance(other, Number):
            self.x = self.x / other
            self.y = self.y / other
            return self
        else:
            raise ValueError("Unsupported division operation")

    def __truediv__(self, other):
        if isinstance(other, Point):
            return Point(self.x / other.x, self.y / other.y)
        elif isinstance(other, (int, float)):
            return Point(self.x / other, self.y / other)
        raise ValueError("Unsupported division operation")

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __ifloordiv__(self, other):
        if isinstance(other, Point):
            self.x = self.x // other.x
            self.y = self.y // other.y
            return self
        elif isinstance(other, Number):
            self.x = self.x // other
            self.y = self.y // other
            return self
        else:
            raise ValueError("Unsupported floor division operation")

    def __floordiv__(self, other):
        if isinstance(other, Point):
            return Point(self.x // other.x, self.y // other.y)
        elif isinstance(other, (int, float)):
            return Point(self.x // other, self.y // other)
        raise ValueError("Unsupported floor division operation")

    def __rfloordiv__(self, other):
        return self.__floordiv__(other)

    def __isub__(self, other):
        if isinstance(other, Point):
            self.x -= other.x
            self.y -= other.y
            return self
        elif isinstance(other, Number):
            self.x -= other
            self.y -= other
            return self
        else:
            raise ValueError('Unsupported subtraction operation')

    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self.x - other.x, self.y - other.y)
        elif isinstance(other, Number):
            return Point(self.x - other, self.y - other)
        raise ValueError('Unsupported subtraction operation')

    def __rsub__(self, other):
        return self.__sub__(other)

    def __iadd__(self, other):
        if isinstance(other, Point):
            self.x += other.x
            self.y += other.y
            return self
        elif isinstance(other, Number):
            self.x += other
            self.y += other
            return self
        else:
            raise ValueError('Unsupported addition operation')

    def __add__(self, other):
        if isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y)
        elif isinstance(other, Number):
            return Point(self.x + other, self.y + other)
        raise ValueError('Unsupported addition operation')

    def __radd__(self, other):
        return self.__add__(other)

    def __imul__(self, other):
        if isinstance(other, Point):
            self.x *= other.x
            self.y *= other.y
            return self
        elif isinstance(other, Number):
            self.x *= other
            self.y *= other
            return self
        else:
            raise ValueError('Unsupported multiplication operation')

    def __mul__(self, other):
        if isinstance(other, Point):
            return Point(self.x * other.x, self.y * other.y)
        elif isinstance(other, Number):
            return Point(self.x * other, self.y * other)
        raise ValueError('Unsupported multiplication operation')

    def __rmul__(self, other):
        return self.__mul__(other)

    def distance(self, other):
        if isinstance(other, Point):
            return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
        raise ValueError('Unsupported distance operation')

    def to_dict(self):
        return {'x': self.x, 'y': self.y}

    def to_array(self):
        return [self.x, self.y]

    def copy(self):
        return Point(self.x, self.y)
