import math


class Vector:
    """ Vector in R^2
    """

    def __init__(self, xy: tuple):
        """ Vector is defined by two points
        """
        self.x, self.y = xy

    @property
    def magnitude(self):
        """ distance between self and other
        """
        return math.sqrt(self.x**2 + self.y**2)

    def __sub__(self, other):
        n_x = other.x - self.x
        n_y = other.y - self.y
        return Vector((n_x, n_y))

    def __add__(self, other):
        n_x = self.x + other.x
        n_y = self.y + other.y
        return Vector((n_x, n_y))

    def scalar_mult(self, scalar):
        n_x = self.x * scalar
        n_y = self.y * scalar
        return Vector((n_x, n_y))

    @property
    def v_hat(self):
        """ get the unit vector
        """
        mag = self.magnitude
        return Vector((self.x / mag, self.y / mag))

    def __repr__(self):
        return "Vector(x: {}, y: {})".format(self.x, self.y)


class LineSegment(Vector):
    """ Line is defined by two vectors
    """

    tolerance = 0.001

    def __init__(self, xy_a: tuple, xy_b: tuple):
        self.vec_a = Vector(xy_a)
        self.vec_b = Vector(xy_b)
        self.as_vec = self.vec_a - self.vec_b


    def _ccw(self, A, B, C):
        return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

    def intersect(self, other):
        return (self._ccw(self.vec_a, other.vec_a, other.vec_b) != self._ccw(self.vec_b, other.vec_a, other.vec_b) and
               self._ccw(self.vec_a, self.vec_b, other.vec_a) != self._ccw(self.vec_a, self.vec_b, other.vec_b))

    def get_endpoints(self) -> list:
        return [(self.vec_a.x, self.vec_a.y), (self.vec_b.x, self.vec_b.y)]

    def __repr__(self):
        return "Line( pt_a: {}, pt_b: {})".format(self.vec_a, self.vec_b)


