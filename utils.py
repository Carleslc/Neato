from log import info
from datetime import datetime
from math import sin, cos, atan2, degrees, radians

def mod(alfa):
    return alfa % 360 if alfa >= 0 else alfa % -360

def abs_alfa(alfa):
    alfa = mod(alfa)
    return alfa + 360 if alfa < 0 else alfa

def sign(x):
    return 1 if x >= 0 else -1

def for_each(values, block):
    """ Apply block for each value in values """
    for value in values:
        block(value)

def print_each(values):
    """ Print each value in values """
    for_each(values, info)

def separator():
    return '--- %s ---' % datetime.now().time()

def is_zero(x, limit=0.001):
    return abs(x) < limit

def medianOfThree(a, b, c):
    return a + b + c - min(a, b, c) - max(a, b, c)

def mean_angle(alfa1, alfa2):
    return degrees(atan2((sin(radians(alfa1))+sin(radians(alfa2)))/2,(cos(radians(alfa1))+cos(radians(alfa2)))/2))