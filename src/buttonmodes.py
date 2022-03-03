from utiltools import *
from physic import *
from pyglet.window import mouse as mou
from pyglet.window import key
from interface import *
import typing
from modeenum import *
class ClickHandler:
    runmode: RunModes = RunModes.pause
    mode: UpdateModes = UpdateModes.Stay
    buttons: list
    pend = None
    spr = None
    def __init__(self, physic: Engine, cam: Camera, ui):
        self.ui = ui
        self.camera = cam
        self.buttons = []
        self.physic = physic

    def add(self, buttons: list):
        self.buttons = buttons
    def updateRun(self, mode: RunModes):
        self.runmode = mode
        self.physic.changeMode(mode)
    def on_mouse_press(self, x, y, button, modifiers):
        vec = Vector2d(x, y)

        match self.runmode:
            case RunModes.run:
                ...
                print(self.runmode)
            case RunModes.pause:
                ...
        for i in self.buttons:
            if vec in i:
                match i.onClick:
                    case UpdateModes():
                        self.mode = i.onClick if self.mode != i.onClick else UpdateModes.Stay
                        if self.mode == 0:
                            i.soft()
                        else:
                            for j in self.buttons:
                                j.soft()
                            i.glw()
                        self.updateRun(RunModes.pause)
                        break
                    case RunModes():
                        self.updateRun(RunModes.run if self.runmode == RunModes.pause else RunModes.pause)
                        for j in self.buttons:
                            if j != i:
                                j.soft()
                        #self.mode = UpdateModes.Stay
                        i.revglw()

        else:
            if vec in self.UI or self.runmode == RunModes.run:
                return
            centre = Vector2d(x, y) - self.camera.centre
            match self.mode, button, modifiers:
                case _, mou.LEFT, 17:
                    phy = self.physic.getByClick(centre)
                    for i in phy:
                        match i:
                            case Point():
                                self.ui.setParamFrame(i)
                                return
                    for i in phy:
                        match i:
                            case Pendulum():
                                self.ui.setParamFrame(i)
                                return
                    for i in phy:
                        match i:
                            case Spring():
                                self.ui.setParamFrame(i)
                                return
                case UpdateModes.Stay, _, _:
                    ...
                case UpdateModes.Static, mou.LEFT, _:
                    self.physic.addFixed(centre, PointSize)
                case UpdateModes.Mobile, mou.LEFT, _:
                    self.physic.addMotionable(centre, flt(5), PointSize)
                case UpdateModes.Pendulum, mou.LEFT, _:
                    self.pend = centre
                case UpdateModes.Spring, mou.LEFT, _:
                    self.spr = centre


    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        match buttons:
            case mou.RIGHT | mou.MIDDLE:
                self.camera.translate(dx, dy)

    def on_mouse_release(self, x, y, button, modifiers):
        vec = Vector2d(x, y) - self.camera.centre
        if Vector2d(x, y) in self.UI:
            return
        match self.mode, button, self.runmode:
            case UpdateModes.Pendulum, mou.LEFT, RunModes.pause if self.pend:
                response = self.physic.addPendulum(self.pend, vec)
                if response == 0:
                    self.physic.addPendulum(vec, self.pend)
                self.pend = None
            case UpdateModes.Spring, mou.LEFT, RunModes.pause if self.spr:
                response = self.physic.addSpring(self.spr, vec)
                if response == 0:
                    self.physic.addSpring(vec, self.spr)
                self.spr = None
    def giveFramesData(self, ui):
        self.UI = ui
