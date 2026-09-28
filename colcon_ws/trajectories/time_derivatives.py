"""Small second-order automatic differentiator for trajectory equations.

The editable position equations can use +, -, *, / by a number, integer
powers, and the sin/cos functions below. No extra Python package is needed.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TimeValue:
    value: float
    velocity: float = 0.0
    acceleration: float = 0.0

    @classmethod
    def time(cls, seconds):
        return cls(seconds, 1.0, 0.0)

    def __add__(self, other):
        other = as_time_value(other)
        return TimeValue(self.value + other.value,
                         self.velocity + other.velocity,
                         self.acceleration + other.acceleration)

    __radd__ = __add__

    def __neg__(self):
        return TimeValue(-self.value, -self.velocity, -self.acceleration)

    def __sub__(self, other):
        return self + -as_time_value(other)

    def __rsub__(self, other):
        return as_time_value(other) - self

    def __mul__(self, other):
        other = as_time_value(other)
        return TimeValue(self.value * other.value,
                         self.velocity * other.value + self.value * other.velocity,
                         self.acceleration * other.value +
                         2 * self.velocity * other.velocity +
                         self.value * other.acceleration)

    __rmul__ = __mul__

    def __truediv__(self, number):
        if isinstance(number, TimeValue):
            raise TypeError('divide by a constant number, not a time-dependent value')
        return self * (1 / number)

    def __pow__(self, exponent):
        if not isinstance(exponent, int) or exponent < 0:
            raise TypeError('use a nonnegative integer power')
        if exponent == 0:
            return TimeValue(1.0)
        value = self.value**exponent
        velocity = exponent * self.value**(exponent - 1) * self.velocity
        acceleration = exponent * self.value**(exponent - 1) * self.acceleration
        if exponent >= 2:
            acceleration += (exponent * (exponent - 1) *
                             self.value**(exponent - 2) * self.velocity**2)
        return TimeValue(value, velocity, acceleration)


def as_time_value(value):
    return value if isinstance(value, TimeValue) else TimeValue(float(value))


def sin(value):
    value = as_time_value(value)
    return TimeValue(math.sin(value.value),
                     math.cos(value.value) * value.velocity,
                     math.cos(value.value) * value.acceleration -
                     math.sin(value.value) * value.velocity**2)


def cos(value):
    value = as_time_value(value)
    return TimeValue(math.cos(value.value),
                     -math.sin(value.value) * value.velocity,
                     -math.sin(value.value) * value.acceleration -
                     math.cos(value.value) * value.velocity**2)
