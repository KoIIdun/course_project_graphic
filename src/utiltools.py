import numpy as np
import pyglet.window
from multimethods import multimethod as overload
import pyglet as pg
from pyglet.gl import *
from utiltools import *
from typing import Tuple
import collections.abc as abc
from constants import *
class Vector2d:
    __coords = NULL
    @overload(PRIM, PRIM)
    def __init__(self, x, y, *_):
        self.__coords = np.array([x, y])
    @overload(np.ndarray)
    def __init__(self, coords):
        self.__coords = coords
    @overload(list)
    def __init__(self, coords):
        self.__init__(np.array(coords))
    @overload(object)
    def __init__(self, vector):
        if isinstance(vector, self.__class__):
            self.__init__(vector.getCoords())
        else:
            raise ValueError("No matching method")
    def __getitem__(self, index):
        return self.__coords[index]
    def __str__(self) -> str:
        return f"({self[0]}, {self[1]})"
    def set(self, x, y):
        self.__coords = np.array([x, y])
    def __add__(self, other):
        return Vector2d(self.__coords + other.getCoords())
    def __neg__(self):
        return Vector2d(-self.__coords)
    def __sub__(self, other):
        return Vector2d(self.__coords - other.getCoords())
    def __floordiv__ (self, num: int):
        return Vector2d(self.__coords // num)
    def __truediv__(self, num):
        return Vector2d(self.__coords / num)
    def getCoords(self) -> np.ndarray:
        return self.__coords
    def len(self):
        return flt(np.linalg.norm(np.int32(self.__coords)))
    def ortoLeft(self):
        return Vector2d([-self.__coords[1], self.__coords[0]])
    def __mul__(self, other):
        match other:
            case Vector2d():
                x, y = self.__coords
                ox, oy = other.getCoords()
                return y * ox - x * oy
            case _:
                return Vector2d(self.__coords * other)
                # raise TypeError("mul vector on vector")
                # return
    def ortoRight(self):
        return Vector2d([self.__coords[1], -self.__coords[0]])
    def unificate(self):

        vec = np.hypot(np.array(self.__coords[0], dtype=flt),
                       np.array(self.__coords[1], dtype=flt))#np.sqrt(np.sum(self.__coords ** 2))
        return Vector2d(self.__coords / vec)
    def toLen(self, k):
        vec = self.unificate()
        return Vector2d(vec.getCoords() * k)
    def vecMul(self, b):
        x, y = self.__coords
        return np.array([0, 0, x * b[1] - y * b[0]])
Vector2d.NULL = Vector2d(NULL)
VECPRIM = Vector2d | list | np.ndarray

class Bounds:
    __top: Vector2d
    __bot: Vector2d
    __centre: Vector2d
    @overload(VECPRIM, VECPRIM)
    def __init__(self, bot, top):
        self.__width, self.__height = self.__shapes = Vector2d(top) - Vector2d(bot)
        if np.any(0 > self.__shapes.getCoords()):
            raise ValueError("Bounds shapes wrong")
        self.__top = Vector2d(top)
        self.__bot = Vector2d(bot)
        self.__centre = self.__bot + self.__shapes // 2
    @overload(VECPRIM, PRIM, PRIM)
    def __init__(self, bot, width, height):
        bot = Vector2d(bot)
        vec = bot + Vector2d(width, height)
        self.__init__(bot, vec)
    def __contains__(self, item: Vector2d):
        itemCoords = item.getCoords()
        return np.all(self.__top <= itemCoords <= self.__bot)
    def __str__(self) -> str:
        return f"Bounds: {{{self.__bot}, w: {self.__width}, h: {self.__height}}}"
    def translate(self, tx, ty):
        tvec = Vector2d(tx, ty)
        self.__top += tvec
        self.__bot += tvec
        self.__centre += tvec
    def reShape(self, width, height):
        self.__width, self.__height = self.__shapes = Vector2d(width, height)
        diam = self.__shapes // 2
        self.__top = self.__centre + diam
        self.__bot = self.__centre - diam

    @property
    def top(self): return self.__top
    @property
    def bot(self): return self.__bot
    @property
    def centre(self): return self.__centre
    @property
    def width(self): return self.__width
    @property
    def height(self): return self.__height
    @property
    def shapes(self): return self.__shapes


class Camera(Bounds):
    def setup(self):
        #glLoadIdentity()



        glTranslatef(*self.centre[:], 0)

#glTranslatef(view_x, 0.0, -zoom) # Move left 1.5 units and into the screen 6.0
        #glTranslatef(self.centre()[0], self.centre()[1], 0) # Move left 1.5 units and into the screen 6.0
    def reset(self):
        self.rotx, self.roty, self.rotz = 0, 0, 0

    #pass

def createDefaultWindow(config=None):
    win = pg.window.Window(*STANDART_WINDOW_SHAPE, resizable=True, vsync=True, config=config)
    win.set_minimum_size(200, 300)
    return win