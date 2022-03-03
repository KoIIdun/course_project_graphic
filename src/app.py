import pyglet as pg
from pyglet.gl import *
from constants import *
from pyglet.window import *
from pyglet.window import mouse
from interface import *

import os
from utiltools import *
import numpy as np
from physic import *
from math import *
from primitives import *
from buttonmodes import *
from modeenum import *
import glooey
config = None # pyglet.gl.Config(sample_buffers=1, samples=4)
WINDOW = createDefaultWindow(config=config)
WINDOW.config.alpha_size = 8
CAMERA = Camera([0, 0], *WINDOW.get_size())
PHYSIC = Engine()
UI = UIFactory.bottomLineUI(50)
HANDLE = ClickHandler(PHYSIC, CAMERA, UI)
UI.connect(HANDLE)
UI.getButtons()
BACKGROUND_COLOR = SEMI_WHITE_COLOR
FPS = FPSDisplay(WINDOW)

@WINDOW.event
def on_draw():
    WINDOW.clear()
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glViewport(0, 0, *WINDOW.get_size())
    gluOrtho2D(0, WINDOW.width, 0, WINDOW.height)

    UI.draw()
    #glTranslatef(*(-CAMERA.centre)[:], 0)
    BATCH.draw()
    #glTranslatef(*(CAMERA.centre)[:], 0)
    CAMERA.setup()
    glMatrixMode(GL_MODELVIEW)
    PHYSIC.draw()



    glClearDepth(1.0)               # Depth buffer setup
    glEnable(GL_DEPTH_TEST)         # Enables depth testing
    glDepthFunc(GL_LEQUAL)          # The type of depth test to do

    return pyglet.event.EVENT_HANDLED

@WINDOW.event
def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    HANDLE.on_mouse_drag(x, y, dx, dy, buttons, modifiers)

@WINDOW.event
def on_key_press(symbol, modifiers):
    global CAMERA
    shift = 5
    match symbol:
        case key.R:
            CAMERA.reset()
@WINDOW.event
def on_mouse_press(x, y, button, modifiers):
    HANDLE.on_mouse_press(x, y, button, modifiers)
@WINDOW.event
def on_mouse_release(x, y, button, modifiers):
    HANDLE.on_mouse_release(x, y, button, modifiers)
@WINDOW.event
def on_resize(width, height):
    win = (width, height)
    CAMERA.reShape(*win)
    UI.updateWindow(*win)
    glViewport(0, 0, *win)
    glMatrixMode(GL_PROJECTION)
    gluOrtho2D(0, width, 0, height)
    glMatrixMode(GL_MODELVIEW)
def tests():
    global GUI
    a = Vector2d(1, 3)
    b = Vector2d(1, 1)
    bounds = Bounds([0,0], [30,30])
#    a = Pendulum(Fixed([0,1], 0.1), Motionable([0,1], 0.1))
    c = b.len()
    i = UIFactory.bottomLineUI(300)
    #print(i)
    # button1 = Button(bounds)
    # # button2 = Button([30, 30], [90, 90])
    # print(WINDOW.get_size())
    # a = ParamShapes("100%", "100%", WINDOW.get_size())
    # print(a)
    # WINDOW.maximize()
    # a.update(WINDOW.get_size())
    # print(a, WINDOW.get_size())
    #WINDOW.maximize()


BATCH = pyglet.graphics.Batch()
group = pyglet.graphics.Group()
GUI = glooey.Gui(WINDOW, batch=BATCH, group=group)
UI.bindGUI(GUI)
if __name__ == '__main__':
    tests()
    d = Fixed([0, 0], flt(39.0))
    PHYSIC.addPrimitive(d)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT)
    glClearColor(*np.array(BACKGROUND_COLOR) / 256, 255)
    pg.clock.schedule(PHYSIC.updateTime)
    #pg.clock.schedule_interval(PHYSIC.updateTime, 0.05)
    pg.clock.get_default()
    pg.app.run()
