# models.py
from dataclasses import dataclass

@dataclass
class Point3:
    x: float
    y: float
    z: float

@dataclass
class Marker:
    id: int
    position: Point3
    oriental: Point3

def p3_from_list(l):
    return Point3(l[0], l[1], l[2])