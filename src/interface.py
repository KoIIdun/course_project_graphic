#interface.py
import itertools

from multimethods import multimethod as overload

from buttonmodes import ClickHandler
from modeenum import *
from utiltools import *
from constants import *
from buttonmodes import *
from glooey.text import Form
from glooey.containers import HBox
from physic import *
import glooey
class Representation:
    isPercent: bool = None
    percent: int = None
    value: int = None
    shape: int = None

    def __init__(self, value: str | int, shape):
        checkPercent = (lambda x: x in range(101))
        isPersent = (lambda x: x[-1] == "%"
                               and checkPercent(int(x[:-1])))
        match value:
            case str(persent) if isPersent(persent) and shape:
                self.isPercent = True
                self.percent = int(persent[:-1])
                self.shape = shape
                self.refresh()
            case int(pixel):
                self.isPercent = False
                self.value = pixel
            case _:
                raise ValueError("Argument is bad")

    def updateShape(self, win: int):
        self.shape = win
        self.refresh()

    def refresh(self):
        if self.isPercent:
            self.value = self.shape * self.percent // 100

    def __str__(self):
        return str(self.value)


class ParamShapes:
    windowShapes: SHAPE
    __valError = ValueError("Cannot create Pixels")
    xshape: Representation
    yshape: Representation

    def __init__(self, x, y, winShape):
        self.windowShapes = winShape
        self.xshape = Representation(x, winShape[0])
        self.yshape = Representation(y, winShape[1])

    def __refreshPixels(self):
        self.xshape.updateShape(self.windowShapes[0])
        self.yshape.updateShape(self.windowShapes[1])

    def updateShapeGlobal(self, win: SHAPEI):
        self.windowShapes = win
        self.__refreshPixels()

    def __str__(self):
        return f"Pixels:{{{self.xshape}, {self.yshape}}}"

class ParamPosition:
    windowShapes: SHAPE
    xcoord: Representation
    ycoord: Representation

    def __init__(self, x, y, winShape):
        self.windowShapes = winShape
        self.xcoord = Representation(x, winShape[0])
        self.ycoord = Representation(y, winShape[1])

    def __refreshCoordPixels(self):
        self.xcoord.updateShape(self.windowShapes[0])
        self.ycoord.updateShape(self.windowShapes[1])

    def updateCoordGlobal(self, win: SHAPEI):
        self.windowShapes = win
        self.__refreshCoordPixels()
    ...

class Clickable:
    hitbox: Bounds
    globalVec: Vector2d
    def __init__(self, hitbox: Bounds):
        self.hitbox = hitbox
    def __contains__(self, item: Vector2d):
        ...
    def onClick(self):
        ...

class Hidable:
    showStatus = False | True | 2 | 3
    #allways never
    show: showStatus
    def __init__(self, show = True) -> None:
        self.show = show
    def hide(self):

        if self.show < 2:
            self.show = False
    def show(self):
        if self.show < 2:
            self.show = True

class Frame(ParamShapes, Bounds, Hidable, ParamPosition):
    underframes: list
    def __init__(self,
                 coords: tuple,
                 shapes: tuple,
                 winShapes: SHAPEI,
                 color: COLOR,
                 show = True) -> None:
        self.globalVec = Vector2d([0, 0])
        Hidable.__init__(self, show)
        super().__init__(*shapes, winShapes)
        self.color = color
        w, h = self.xshape.value, self.yshape.value
        ParamPosition.__init__(self, *coords, winShapes)
        bot = Vector2d(self.xcoord.value, self.ycoord.value)

        vec = bot + Vector2d(w, h)
        Bounds.__init__(self, bot, vec)
        self.underframes = []
    def hide(self):
        Hidable.hide(self)

        for i in self.underframes:
            i.hide()
    def show(self):
        Hidable.show(self)
        for i in self.underframes:
            i.show()



    def updateShape(self, win: SHAPEI, globalVec: Vector2d):
        self.globalVec = globalVec
        super().updateShapeGlobal(win)
        ParamPosition.updateCoordGlobal(self, win)
        self.reShape(self.xshape.value, self.yshape.value)
        for i in range(len(self.underframes)):
            self.underframes[i].updateShape((self.xshape.value, self.yshape.value), self.globalVec + self.bot)
    def reShape(self, width, height):
        bot = Vector2d(self.xcoord.value,
                       self.ycoord.value)
        top = Vector2d(width, height) + bot
        super(ParamShapes, self).__init__(bot, top)
    def __contains__(self, item: Vector2d):
        item = item - self.globalVec
        itemCoords = item.getCoords()
        return np.any(np.all([self.bot.getCoords() <= itemCoords, itemCoords <= self.top.getCoords()]))
    def getButtons(self) -> list:
        lst = []
        for i in self.underframes:
            lst.append(i if isinstance(i, Button) else i.getButtons())

        return lst
    def drawOwn(self, color):


        z = 1
        if self.show in (1, 2):
            glBegin(GL_QUAD_STRIP)
            glColor3b(*color)
            glVertex3i(*self.bot[:], z)
            glVertex3i(self.bot[0], self.top[1], z)
            glVertex3i(self.top[0], self.bot[1], z)
            glVertex3i(*self.top[:], z)
            glEnd()
    def draw(self):
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        self.drawOwn(self.color)
        glTranslatef(*(self.bot[:]), 0)
        for i in self.underframes:
            i.draw()
        glTranslatef(*(-self.bot[:]), 0)
    def bindObject(self, draw):
        self.underframes.append(draw)
        self.underframes[-1].updateShape((self.xshape.value, self.yshape.value), self.globalVec + self.bot)
    def __str__(self):
        return f"Frame:{{bot: {self.bot}, shapes: {self.shapes}}}"

    # def bindButton(self, button: Button):
    #     .buttons.append(button)
    #
class Glowa(Frame):
    glow: bool = False
    def draw(self):
        Frame.draw(self)
        if self.glow:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            self.drawOwn((200,200,200))
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    def glw(self):
        self.glow = True
    def soft(self):
        self.glow = False
    def revglw(self):
        self.glow = not self.glow

class Button(Glowa, Clickable):
    texturbox: Bounds
    def __init__(self, mode, *args):
        self.onClick = mode
        Frame.__init__(self, *args)
    def bindObject(self, draw):
        pass

    def __str__(self) -> str:
        return f"Button: {{" \
               f"texturebox: {self.globalVec}}}"

    # @overload(list, list, str, list, list)
    # def __init__(self, top, bot, tp="", hittop=None, hitbot=None):
    #     hitbot = None if not hitbot else Vector(*hitbot)
    #     hittop = None if not hitbot else Vector(*hittop)
    #     self.__init__(Vector(*top), Vector(*bot), tp, hitbot, hittop)

class InputFrame(glooey.Frame):
    class Base(glooey.Background):
        custom_color = '#76b6c4'
    Decoration = Base
    custom_width_hint = 210
    custom_alignment = 'top right'
class InputBlock(HBox):
    ...

class TextInput(Form):
    custom_alignment = 'fill'
    class Base(glooey.Background):
        custom_color = '#ffffff'
    def __init__(self, name, phy):
        super().__init__(str(getattr(phy, name)))
        self.name = name
        self.phy = phy
class MyTitle(glooey.Label):
    custom_color = '#ffffff'
    custom_font_size = 12
    custom_alignment = 'center'
    custom_bold = True
class MyLabel(glooey.Label):
    custom_color = '#babdb6'
    custom_font_size = 10
    custom_alignment = 'center'
class MyButton(glooey.Button):
    Foreground = MyLabel
    custom_alignment = 'fill'
    phy: PhyPrimitive

    class Base(glooey.Background):
        custom_color = '#204a87'

    class Over(glooey.Background):
        custom_color = '#3465a4'

    class Down(glooey.Background):
        custom_color = '#729fcff'
    def __init__(self, text, input: TextInput, phy: PhyPrimitive):
        super().__init__(text)
        self.input = input
        self.phy = phy

    def on_click(self, widget):
        #        input.
        Interface.inpMode = True
        #print(flt(self.input.get_text()))
        self.phy.updateAttr(self.input.name, flt(self.input.get_text()))

class BoxFactory:
    namedict = {"mass":"масса",
                "radius":"радиус",
                "ph":"фи",
                "k":"k",
                "g":"g",
                "velX": "Vx",
                "velY": "Vy",
                "length":"длина"}
    @staticmethod
    def InputBox(phy: PhyPrimitive = None) -> InputFrame:
        frame = InputFrame()
        vbox = glooey.VBox()
        vbox.padding = 5
        #
        vbox.custom_alignment = 'center'
        title = MyTitle("Параметры")
        vbox.add(title)
        for att in phy.phyparams:
            nam = BoxFactory.namedict[str(att)]
            name = MyTitle(nam)
            hbox = InputBlock()
            hbox.padding = 5
            tinput = TextInput(str(att), phy)
            button = MyButton(">", tinput, phy)
            hbox.add(name)
            hbox.add(tinput)
            hbox.add(button)
            vbox.add(hbox)
        frame.add(vbox)
        return frame

class Interface:
    frames: list[Frame]
    windowShape: SHAPEI
    buttons: list[Button]
    handle: ClickHandler
    paramFrame: InputFrame = None

    def __init__(self, windowShape):
        self.buttons = []
        self.frames = []
        self.handler = None
        Interface.windowShape = windowShape
    def bindGUI(self, gui):
        self.gui = gui
    def setParamFrame(self, phy: PhyPrimitive):
        if self.paramFrame != None:
            self.gui.remove(self.paramFrame)
        self.paramFrame = BoxFactory.InputBox(phy)
        self.gui.add(self.paramFrame)
    def updateWindow(self, x, y):
        Interface.windowShape = win = (x, y)
        for index in range(len(self.frames)):
            self.frames[index].updateShape(win, Vector2d(NULL))

    def bindFrame(self, frame: Frame):
        self.frames.append(frame)

    def connect(self, handler):
        self.handler = handler
        self.handler.giveFramesData(self)

    def getButtons(self):
        for i in self.frames:
            self.buttons = list(itertools.chain(self.buttons, *i.getButtons()))
        self.handler.add(self.buttons)
    def draw(self):
        # glMatrixMode(GL_MODELVIEW)

        for frame in self.frames:
            frame.draw()

    def __str__(self):
        return f"Interface: {''.join(list(map(str, self.frames)))}"

    def __contains__(self, item: Vector2d):
        return any([item in i for i in self.frames])

class buttonsFactory:
    @staticmethod
    def addPoint(mode, position, paramXShape):
        button = Button(mode, position, paramXShape, STANDART_WINDOW_SHAPE, randomColor())
        return button
    @staticmethod
    def fixedAdd(position, paramXShape):
        button = Button(UpdateModes.Static, position, paramXShape, STANDART_WINDOW_SHAPE, randomColor())
        return button
    @staticmethod
    def movableAdd(position, paramXShape):
        button = Button(UpdateModes.Mobile, position, paramXShape, STANDART_WINDOW_SHAPE, randomColor())
        return button
    @staticmethod
    def springAdd(position, paramXShape):
        button = Button(UpdateModes.Spring, position, paramXShape, STANDART_WINDOW_SHAPE, randomColor())
        return button
    @staticmethod
    def pendulumAdd(position, paramXShape):
        button = Button(UpdateModes.Pendulum, position, paramXShape, STANDART_WINDOW_SHAPE, randomColor())
        return button
    @staticmethod
    def switchRun(position, paramXShape):
        button = Button(RunModes.run, position, paramXShape, STANDART_WINDOW_SHAPE, randomColor())
        return button
class FrameFactory:
    @staticmethod
    def centreBox():
        centre = Frame(("40%", 0), ("20%", "100%"), STANDART_WINDOW_SHAPE, randomColor(), 3)
        return centre

    @staticmethod
    def bottomLine(paramYShape):
        bottom = Frame([0, 0], ("100%", paramYShape), STANDART_WINDOW_SHAPE, AQUA_COLOR)
        return bottom

    @staticmethod
    def buttonsBox():
        n = 1
        centre = FrameFactory.centreBox()
        shiftparam = 7
        shift = shiftparam
        edge = 36
        buttons = [buttonsFactory.fixedAdd((shift, shiftparam), (edge, edge))]
        shift += edge + shiftparam
        buttons.append(buttonsFactory.movableAdd((shift, shiftparam), (edge, edge)))
        shift += edge + shiftparam
        buttons.append(buttonsFactory.pendulumAdd((shift, shiftparam), (edge, edge)))
        shift += edge + shiftparam
        buttons.append(buttonsFactory.springAdd((shift, shiftparam), (edge, edge)))
        shift += edge + shiftparam
        buttons.append(buttonsFactory.switchRun((shift, shiftparam), (edge, edge)))
        for i in buttons:
            centre.bindObject(i)
        return centre


class UIFactory:
    @staticmethod
    def bottomLineUI(height):
        frame = FrameFactory.bottomLine(height)
        frame.bindObject(FrameFactory.buttonsBox())
        int3 = Interface(STANDART_WINDOW_SHAPE)
        int3.bindFrame(frame)
        return int3