import math

import numpy as np
import sympy

from utiltools import *
from constants import *
from multimethods import multimethod as overload
from primitives import *
from modeenum import *
from sympy import *
from sympy.physics.mechanics import *
from sympy import sqrt as symsqrt
from sympy import sin
from sympy import atan2 as atn
import math as mth

LEVEL = 0
PointSize = 25
G = flt(9.80665)
two = flt(2.0)
class Counter:
    count = 0
    @staticmethod
    def plus():
        Counter.count += 1
        return Counter.count - 1
class Translatable:
    def __init__(self, centre):
        self.centre = Vector2d(centre)

    def translate(self, tvec: VECPRIM):
        tvec = Vector2d(tvec)
        self.centre += tvec
    def setCentre(self, vec: VECPRIM):
        self.centre.set(*vec[:])

class Drawable(Translatable):
    def __init__(self, primitive: int, centre: VECPRIM, color: COLOR = None):
        self.color = randomColor() if not color else color
        self.primitiveId = primitive
        Translatable.__init__(self, centre)
    def draw(self):
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glTranslatef(*self.centre, LEVEL)
        glColor3b(*self.color)
        # glEnable(GL_LINE_SMOOTH)
        # glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glCallList(self.primitiveId)
        # glDisable(GL_LINE_SMOOTH)
        # glFlush()

        glTranslatef(*(-self.centre), LEVEL)

class Bond:
    pass

class Bondable(Bond):
    def __init__(self):
        self.bonds = []
    def connect(self, target: Bond):
        self.bonds.append(target)
        target.connect(self)

class PhyPrimitive(Drawable):
    phyparams: list[str]

    def __init__(self, primitive: int, centre: VECPRIM, color: COLOR = None):
        super().__init__(primitive, centre, color)
        self.runphi = None
        self.pot = None
        self.kin = None

    def update(self):
       pass
    def updateAttr(self, name, var):
        setattr(self, name, var)
        try:
            setattr(self, name, var)
            self.update()
        except:
            print("Cannot set "+str(name)+" "+str(var))

class Point(Bondable, PhyPrimitive):
    phyparams = ["mass", "radius"]
    mass: flt
    radius: flt
    g: flt = G
    def __init__(self, centre: VECPRIM, mass: flt, radius: flt, g):
        Bondable.__init__(self)
        self.systemated = False
        self.g = g
        self.spring = 0
        self.velocity = Vector2d([0, 0])
        self.centre = Vector2d(centre)
        self.pendTop = None
        self.pendBot = None
        self.pends = []
        self.runx = None
        self.runy = None
        self.potential =[self.centre]
        self.radius = radius
        self.mass = mass
        count = Counter.count
        self.v = (dynamicsymbols(f'xv{count}'), dynamicsymbols(f'yv{count}'))
        self.dv = (dynamicsymbols(f'xv{count}',level=1), dynamicsymbols(f'yv{count}',level=1))
        Counter.plus()
        print(Counter.count, "Point")
        self.kin = self.mass * (self.dv[0] ** two + self.dv[1] ** two) / two
        self.pot = self.mass * self.g * self.v[1] if self.mass != np.inf else 0
    def updatePend(self):
        if self.pendTop:
            self.pendTop.updatePos()
            self.pendTop.update()

    def __contains__(self, vec: VECPRIM):
        return (self.centre - Vector2d(vec)).len() <= self.radius
    def systemate(self):
        self.systemated = True
    def update(self):
        Drawable.__init__(self, Primitives.getCircle(self.radius, 100), self.centre)
        self.kin = self.mass * (self.dv[0] ** two + self.dv[1] ** two) / two
        self.pot = self.mass * self.g * self.v[1]
        self.updatePend()
    def setEquation(self, a, b):
        self.equx = a
        self.equy = b
    def updateFromRut(self):
        #print("potential", *self.potential)
        if self.potential != [self.centre]:
            #print(*self.potential)
            acum = Vector2d(0, 0)
            for i in self.potential:
                acum += i
            acum = acum / len(self.potential)
            self.centre.set(*acum[:])
            self.velocity = [0, 0]
        else:
            x, y = self.runx.y[0].evalf(), self.runy.y[0].evalf()
            self.velocity = [self.runx.y[1], self.runy.y[1]]
            #print("kekw", x, y)
            #print(x, y, *self.velocity)
            self.centre.set(x, y)
        self.potential = [self.centre]
            #self.update()

    # def updateFromRut(self):
    #     x, y = self.runx.y[0].evalf(), self.runy.y[0].evalf()
    #     self.potential.append(Vector2d(np.int32(x), np.int32(y)))
    #     print("potential", *self.potential)
    #     #if self.potential != [self.centre]:
    #     #print(*self.potential)
    #     acum = Vector2d(0, 0)
    #     for i in self.potential:
    #         acum += i
    #     acum = acum / len(self.potential)
    #     self.centre.set(*acum[:])
    #     self.velocity = [self.runx.y[1], self.runy.y[1]]
    #     self.potential = []
    def setPotential(self, vec: VECPRIM):
        self.potential.append(vec)
    def next(self, dt, dict):
        self.runx.rename = dict
        self.runy.rename = dict
        self.runx.next(dt)
        self.runy.next(dt)

class Fixed(Point):
    phyparams = ["radius"]
    def __init__(self, centre: VECPRIM, radius: flt):
        Drawable.__init__(self, Primitives.getCircle(radius, 9), centre)
        self.g = flt(0.0)
        Point.__init__(self, centre, np.inf, radius, flt(0))
        self.kin = 0
        self.pot = 0
        self.systemated = True
    def update(self):
        Drawable.__init__(self, Primitives.getCircle(self.radius, 9), self.centre)
        self.kin = 0
        self.pot = 0
    def setRunge(self, time):
        self.runx = RungeKuttaMethod(time,
                                     [self.centre[0], self.velocity[0]],
                                     self.equx / self.mass,
                                     (self.v[0], self.dv[0]))
        self.runy = RungeKuttaMethod(time,
                                     [self.centre[1], self.velocity[1]],
                                     self.equy / self.mass,
                                     (self.v[1], self.dv[1]))
    def updateFromRut(self):
        pass
    def pushF(self, f):
        pass
    def pushSpr(self, a, b):
        pass



class Motionable(Point):
    phyparams = ["mass", "radius", "g"] #"velX", "velY"]
    def __init__(self, centre: VECPRIM, mass: flt, radius: flt):
        Drawable.__init__(self, Primitives.getCircle(radius, 100), centre)
        Point.__init__(self, centre, mass, radius, G)
        self.vel = Vector2d(0, 0)
        self.velY = flt(0)
        self.velX = flt(0)
        self.spring = 0
        self.W = np.array([0.0, 0.0, 0.0])
        self.null()
    def setRunge(self, time):
        self.runx = RungeKuttaMethod(time,
                                     [self.centre[0], self.velocity[0]],
                                     -(self.equx / self.mass -
                                       diff(self.dv[0], t)),
                                     (self.v[0], self.dv[0]))
        self.runy = RungeKuttaMethod(time,
                                     [self.centre[1], self.velocity[1]],
                                     -(self.equy / self.mass -
                                     diff(self.dv[1], t)), # ?????????????????
                                     (self.v[1], self.dv[1]))
    def draw(self):
        Drawable.draw(self)
    def null(self):
        self.f = [Vector2d(0, -1) * self.mass * self.g]
        self.velX, self.velY = self.vel[:]
    def pushF(self, f):
        self.f.append(f)

        #print(self.centre)
    def setNew(self, dt):
        if False: # self.spring == 1:
            print(1)
            print(self.sprvec, self.centre)
            self.vel = self.vel + Vector2d(*self.sprvec) * dt

            self.centre = self.centre + self.vel * 100 * dt
        if len(self.pends) == 0:

            self.a = np.sum(self.f) / self.mass
            self.vel = self.vel + self.a * dt

            self.centre = self.centre + self.vel * 100 * dt
            return

    def pushSpr(self, a, b):
        self.sprvec = (a, b)




def update(self):
        Drawable.__init__(self, Primitives.getCircle(self.radius, 100), self.centre)
        self.kin = self.mass * (self.dv[0] ** two + self.dv[1] ** two) / two
        self.pot = self.mass * self.g * self.v[1]
        self.vel = Vector2d(self.velX, self.velY)
        self.updatePend()

    # def updateFromRut(self):
    #     x, y = self.runx.y[0], self.runy.y[0]
    #     self.velocity = [self.runx.y[1], self.runy.y[1]]
    #
    #     print("mot", x, y, *self.velocity)
    #     self.centre.set(x, y)
DOT = Fixed | Motionable

class Pendulum(PhyPrimitive):
    phyparams = []
    stationary: DOT
    mobile: Motionable
    length: flt
    ph: flt
    wd = 15
    def __init__(self,
                 fixed: Fixed,
                 motionable: Motionable):
        centre = fixed.centre
        self.length = (fixed.centre - motionable.centre).len()
        Drawable.__init__(self, Primitives.getRectangle(self.length, self.wd), centre)
        self.stationary = fixed
        self.mobile = motionable
        self.stationary.pendBot = self
        self.mobile.pendTop = self
        self.mobile.pends.append(self)
        self.ph = flt(0)
        self.W = np.array([0,0,0])
        count = Counter.count
        self.phi = dynamicsymbols(f'phi{count}')
        self.dphi = dynamicsymbols(f'phi{count}', level=1)
        Counter.plus()
        print(Counter.count, "pend", self.color)
        self.update()
    def setNewMot(self, dt):
        mb = self.mobile
        if len(mb.pends) > 1:
            return
        r = self.vec.toLen(self.length) / 100
        powMomentArr = []
        for i in mb.f:
            powMomentArr.append(r.vecMul(i))
        powMoment = sum(powMomentArr)
        inertMoment = mb.mass * (self.length / 100) ** 2
        aW = np.divide(powMoment, inertMoment)
        self.W = self.W + aW * dt
        self.vel = Vector2d(np.cross(self.W, np.array([*r[:], 0.0]),)[:-1])
        # print(self.centre)
        # print(f"------------\n"
        #       f"aW: {aW}, \n"
        #       f"W: {self.W}, \n"
        #       f"vel: {self.vel}, \n"
        #       f"powM: {powMoment}, \n"
        #       f"inM: {inertMoment}, \n"
        #       f"pwa: {powMomentArr}, \n"
        #       f"len: {self.vec.len() - self.length}")
        self.mobile.vel = self.vel
        cnt = mb.centre + self.vel * 100 * dt
        self.mobile.centre = self.stationary.centre + (cnt - self.stationary.centre).toLen(self.length)
    def update(self):
        m = self.mobile.mass
        self.centre = self.stationary.centre
        self.motcentre = self.mobile.centre
        self.vec = self.motcentre - self.centre
        self.mass = m * (self.length / 10) ** two
        self.kin = self.mass * (self.dphi ** two) / two
        self.pot = (1 - cos(self.phi)) * \
                   self.mobile.mass * \
                   self.mobile.g * \
                   (self.length / 8)
        # self.kin = self.mobile.mass * ((mdv[0] ** two + mdv[1] ** two)) / two
        # self.pot = self.mobile.mass * self.mobile.g * (self.phi * mv[0] + self.dphi * mv[1])
        #(1 - cos(atn(mv[0],mv[1])))
        self.vphi = mth.atan2(self.vec[0], self.vec[1])
        a = abs(self.vphi)
        s = np.sign(self.vphi)
        self.vphi = s * (mth.pi - a)
        self.vdphi = 0
    def updatePos(self):
        self.centre = self.stationary.centre
        self.motcentre = self.mobile.centre
        self.vec = self.motcentre - self.centre
    def inFan(self, vec1: VECPRIM, vec2: VECPRIM, point):
        vec1, vec2, point = Vector2d(vec1), Vector2d(vec2), Vector2d(point)
        a = vec1 * point
        b = point * vec2
        return a > 0 and b > 0

    def __contains__(self, vec: VECPRIM):
        vec = Vector2d(vec)
        vector = self.mobile.centre - self.stationary.centre
        left = vector.ortoLeft().toLen(self.wd)
        right = vector.ortoRight().toLen(self.wd)
        stat2 = self.stationary.centre + left
        stat1 = self.stationary.centre + right
        mob1 = self.mobile.centre + right
        mob2 = self.mobile.centre + left
        m2s2 = stat2 - mob2
        m2m1 = mob1 - mob2
        check1 = self.inFan(m2m1,m2s2, vec - mob2)
        s1m1 = mob1 - stat1
        s1s2 = stat2 - stat1
        check2 = self.inFan(s1s2,s1m1, vec - stat1)
        return check1 and check2
    def draw(self):
        npint = lambda x: np.int32(np.floor(x))
        if abs(self.vec.len() - self.length) > self.mobile.radius / 2:
            Primitives.delete(self.primitiveId)
            Drawable.__init__(self, Primitives.getRectangle(npint(self.vec.len()), self.wd), self.centre, self.color)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glTranslatef(*self.centre, LEVEL)
        u = np.arctan2(npint(self.vec[1]), npint(self.vec[0]))
        cosi = np.degrees(np.arctan2(npint(self.vec[1]), npint(self.vec[0])))
#        np.arctan()
        glRotatef(cosi, 0, 0, 1)
        glColor3b(*self.color)
        glCallList(self.primitiveId)
        glRotatef(cosi, 0, 0, -1)
        glTranslatef(*(-self.centre), LEVEL)
    def setEquation(self, a):
        self.equ = a
    def setRunge(self, time):
        self.runphi = RungeKuttaMethod(time,
                                     [self.vphi, self.vdphi],
                                     -(self.equ / self.mass - diff(self.dphi, t)),
                                     self.phi) # ??????
    def updateMobile(self):
        self.centre = self.stationary.centre
        self.vphi = flt(self.runphi.y[0])
        self.dvphi = flt(self.runphi.y[1])
        #print(self.runphi.y[1])
        v = Vector2d([np.sin(self.vphi) * self.length, -np.cos(self.vphi) * self.length])
        self.mobile.setPotential(self.centre + v)
    def updateFromRut(self):
        self.motcentre = self.mobile.centre
        k = self.vphi
        u = (np.cos(self.vphi) * self.length, np.sin(self.vphi) * self.length)

        self.vec = self.motcentre - self.centre
        #print(f"{id(self.stationary)}: {self.motcentre}")
        #u = np.degrees(self.vphi)
    def next(self, dt, dict):
        self.runphi.rename = dict
        self.runphi.next(dt)
        self.updateMobile()


class Spring(PhyPrimitive):
    phyparams = ["k", "length"]
    first: DOT
    second: DOT
    length: flt
    wd = 30
    def __init__(self,
                 a: DOT,
                 b: DOT):
        centre = a.centre
        self.length = (a.centre - b.centre).len()
        Drawable.__init__(self, Primitives.getRectangle(self.length, self.wd), centre)
        self.first = a
        self.second = b
        self.mass = flt(0)
        self.k = flt(50)

        Counter.plus()
        self.second.spring += 1
        self.first.spring += 1
        self.centre = self.first.centre
        self.motcentre = self.second.centre
        self.vec = self.motcentre - self.centre
        #self.update()
    def w2d(self, m):
        w2 = self.k/m
        r = self.vec.len()
        dr = 1 - self.length / r
        return - w2 *dr
    def pushF(self):
        gukF = -(self.vec - self.vec.toLen(self.length)) * self.k / 100
        self.first.pushF(-gukF)
        self.second.pushF(gukF)
        w2d = self.w2d(self.second.mass)
        ax = w2d * self.motcentre[0]
        ay = w2d * self.motcentre[1] - self.second.g
        w2d = self.w2d(self.first.mass)
        bx = w2d * self.motcentre[0]
        by = w2d * self.motcentre[1] - self.first.g
        self.second.pushSpr(ax, ay)
        self.first.pushSpr(bx, by)

    def update(self):
        #m = self.second.mass
        #self.mass = m * (self.length / 10) ** 2

        self.kin = 0
        self.setPot()
    def next(self, dt, f):
        pass
    def setPot(self):
        acum, left, right = [ZERO for i in range(3)]
        match self.first.pendTop, self.second.pendTop:
            case None, None:
                for i in range(2):
                    #print(self.first.v[i] - self.second.v[i])
                    acum += Pow(self.first.v[i] - self.second.v[i], two)
            case Pendulum() as apend, _:
                acum = (apend.length * sin(self.first.v[0]) + apend.centre[0] - self.second.v[0]) ** two + \
                       (apend.length * cos(self.first.v[1]) + apend.centre[1] - self.second.v[1]) ** two
                match self.second:
                    case Fixed():
                        left = self.second.mass * self.second.g * self.second.v[1]
            case Pendulum() as apend, Pendulum() as bpend:
                acum = ((apend.length * sin(self.first.v[0]) + apend.centre[0]) - (bpend.length * sin(self.second.v[0]) + bpend.centre[0])) ** two + \
                       ((apend.length * cos(self.first.v[1]) + apend.centre[1]) - (bpend.length * cos(self.second.v[1]) + bpend.centre[1])) ** two

        acum = Pow(sqrt(acum) - self.length, two)
        self.pot = left + right + self.k * acum / two

    def updatePos(self):
        self.centre = self.first.centre
        self.motcentre = self.second.centre
        self.vec = self.motcentre - self.centre
    def inFan(self, vec1: VECPRIM, vec2: VECPRIM, point):
        vec1, vec2, point = Vector2d(vec1), Vector2d(vec2), Vector2d(point)
        a = vec1 * point
        b = point * vec2
        return a > 0 and b > 0

    def __contains__(self, vec: VECPRIM):
        vec = Vector2d(vec)
        vector = self.second.centre - self.first.centre
        left = vector.ortoLeft().toLen(self.wd)
        right = vector.ortoRight().toLen(self.wd)
        stat2 = self.first.centre + left
        stat1 = self.first.centre + right
        mob1 = self.second.centre + right
        mob2 = self.second.centre + left
        m2s2 = stat2 - mob2
        m2m1 = mob1 - mob2
        check1 = self.inFan(m2m1,m2s2, vec - mob2)
        s1m1 = mob1 - stat1
        s1s2 = stat2 - stat1
        check2 = self.inFan(s1s2,s1m1, vec - stat1)
        return check1 and check2
    def draw(self):
        npint = lambda x: np.int32(np.floor(x))
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glTranslatef(*self.centre, LEVEL)
        cosi = np.degrees(np.arctan2(npint(self.vec[1]), npint(self.vec[0])))

        glRotatef(cosi, 0, 0, 1)

        #glColor4b(*self.color, 0)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        length = self.vec.len()
        if Primitives.tloader.springraw != None:
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, Primitives.tloader.tex.id)
            Primitives.tloader.tex.blit(0, -self.wd // 2, -1, width= length, height=self.wd)
            glDisable(GL_TEXTURE_2D)
        else:
            glCallList(self.primitiveId)
        glRotatef(cosi, 0, 0, -1)
        glTranslatef(*(-self.centre), LEVEL)
        # glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        # glTranslatef(*self.centre, LEVEL)
        # u = np.arctan2(npint(self.vec[1]), npint(self.vec[0]))
        # cosi = np.degrees(np.arctan2(npint(self.vec[1]), npint(self.vec[0])))
        # #        np.arctan()
        # glRotatef(cosi, 0, 0, 1)
        # glColor3b(*self.color)
        # glCallList(self.primitiveId)
        # glRotatef(cosi, 0, 0, -1)
        # glTranslatef(*(-self.centre), LEVEL)


class RungeKuttaMethod:
    def __init__(self, t, y, equation, name) -> None:
        self.t = t
        self.y = y
        self.equ = equation
        self.name = name
        self.rename = None
    def f(self, t, y):
        return [y[1], self.sbs]
    def next(self, dt):
        #print(f"y:                 {self.y}")
        self.sbs = self.equ.subs(self.rename).doit().evalf(3)
        tdt = self.t + dt / two
        yval = lambda y, k: y + k * (dt / two)

        value = lambda k: [yval(self.y[0], k[0]), yval(self.y[1], k[1])]
        k1 = self.f(self.t, self.y)
        k2 = self.f(tdt, value(k1))
        k3 = self.f(tdt, value(k2))
        vk = value(k3)
        vk[0] += dt / two
        vk[1] += dt / two
        k4 = self.f(self.t + dt, vk)
        for i in range(2):
            self.y[i] = (self.y[i] + dt / flt(6.0) * (      k1[i] +
                                                    two * k2[i] +
                                                    two * k3[i] +
                                                          k4[i])
                         ).evalf(3)
        self.t += dt
        #print(self.y[1], k1, k2, k3, k4)

t = Symbol('t')
class Engine:
    objects: list[PhyPrimitive]
    runmode: RunModes = RunModes.pause
    def __init__(self):
        self.time = ZERO
        self.objects = []
    def addPrimitive(self, primitive: PhyPrimitive):
        self.objects.append(primitive)
    def runConf(self):
        for i in self.objects:
            match i:
                case Spring():
                    i.update()
        for i in self.objects:
            match i:
                case Spring():
                    i.updatePos()
        self.kin = 0
        self.pot = 0
        for i in self.objects:
            self.kin += i.kin
            self.pot += i.pot
        self.lagrangian = self.kin - self.pot
        self.equations = []
        for i in self.objects:
            var = lambda a, b: diff(diff(self.lagrangian, a), t) - diff(self.lagrangian, b)
            match i:
                case Fixed() | Motionable():
                    q, dq = i.v, i.dv
                    for j in range(2):
                        # u = diff(self.lagrangian, dq[j])
                        # u1 = diff(self.lagrangian, q[j])
                        self.equations.append(diff(diff(self.lagrangian, dq[j]), t) - diff(self.lagrangian, q[j]))
                    i.setEquation(*self.equations[-2:])
                    i.setRunge(self.time)
                case Pendulum():
                    #u = q - i.stationaty
                    q, dq = i.phi, i.dphi
                    self.equations.append(var(dq, q))
                    i.setEquation(self.equations[-1])
                    i.setRunge(self.time)
        #print(self.equations)
    def next(self, dt):
        if dt > 0.1:
            return
        for i in self.objects:
            match i:
                case Spring():
                    i.pushF()
        for i in self.objects:
            match i:
                case Pendulum():
                    i.setNewMot(dt)
        for i in self.objects:
            match i:
                case Motionable():
                    i.setNew(dt)
        for i in self.objects:
            match i:
                case Motionable():
                    i.null()
                case Pendulum() | Spring():
                    i.updatePos()
    # def next(self, dt):
    #     f = []
    #     for i in self.objects:
    #         match i:
    #             case Point():
    #                 for j in range(2):
    #                     f.append((i.runx.name[j], i.runx.y[j]))
    #                     f.append((i.runy.name[j], i.runy.y[j]))
    #
    #             case Pendulum():
    #                 # print(i.vec.unificate())
    #                 # print(self.lagrangian)
    #                 # dir = i.vec.unificate()
    #                 # dir = dir.ortoLeft()
    #                 # f.append((i.phi, dir[0]))
    #                 # f.append((i.dphi, dir[1]))
    #                 for j in range(2):
    #                     f.append((i.runphi.name, i.runphi.y[j]))
    #                 #f.append()
    #     for i in self.objects:
    #         i.next(dt, f)
    #     for i in self.objects:
    #         match i:
    #             case Point():
    #                 i.updateFromRut()
    #     for i in self.objects:
    #         match i:
    #             case Pendulum():
    #                 #i.updatePos()
    #                 #continue
    #                 i.updateFromRut()
    #     for i in self.objects:
    #         match i:
    #             case Spring():
    #                 i.updatePos()


        # for i in self.objects:
        #     i.
    def updateTime(self, dt):
        dt = flt(dt)
        if self.runmode == RunModes.run:
            self.next(dt)
        self.time += dt
        #print(self.runmode)
    def draw(self):
        for i in self.objects:

            i.draw()
    def changeMode(self, mode: RunModes):
        if self.runmode == mode:
            return
        match mode:
            case RunModes.run:
                ...
                #self.runConf()
            case RunModes.pause:
                ...
        self.runmode = mode
        #i.update()
            # match i:
            #     case Motionable():
            #         i.translate([5, 0])
    # def changePropertiesForm(self, centre: VECPRIM):
    #     sorter = lambda y: sorted(self.primitiveOnPoint(y), key=lambda x: (x.centre - y).len())
    #     phy = sorter(centre)
    #     for i in phy:
    #         match i:
    #             case Point:
    #                 self.ui
    def getByClick(self, centre: VECPRIM):
        sorter = lambda y: sorted(self.primitiveOnPoint(y), key=lambda x: (x.centre - y).len())
        return sorter(centre)
    def addFixed(self, centre: VECPRIM, radius: flt):
        if self.checkCentres(centre):
            fixed = Fixed(centre, radius)
            self.addPrimitive(fixed)
    def addMotionable(self, centre: VECPRIM, mass: flt, radius: flt):
        if self.checkCentres(centre):
            motionable = Motionable(centre, mass, radius)
            self.addPrimitive(motionable)
    def addPendulum(self, centre: VECPRIM, point: VECPRIM):
        sorter = lambda y: sorted(self.primitiveOnPoint(y), key=lambda x: (x.centre - y).len())
        #print((self.primitiveOnPoint(centre)[0].centre - centre).len())#.sort(key=lambda x: (x.centre - centre).len()))
        lstfix = sorter(centre)
        lstmot = sorter(point)
        for i in lstfix:
            match i:
                case Fixed() | Motionable():
                    fix = i
                    break
        else:
            print("Cannot create Pendulum")
            return 0
        for i in lstmot:
            match i:
                case Motionable():
                    mot = i
                    break
        else:
            print("Cannot create Pendulum")
            return 0
        if True:
            pendulum = Pendulum(fix, mot)
            mot.systemate()
            self.addPrimitive(pendulum)
        else:
            print("Cannot create Pendulum (system not fixed)")
            return 0
        return 1
    def addSpring(self, centre: VECPRIM, point: VECPRIM):
        sorter = lambda y: sorted(self.primitiveOnPoint(y), key=lambda x: (x.centre - y).len())
        #print((self.primitiveOnPoint(centre)[0].centre - centre).len())#.sort(key=lambda x: (x.centre - centre).len()))
        lstfix = sorter(centre)
        lstmot = sorter(point)
        for i in lstfix:
            match i:
                case Point():
                    fix = i
                    break
        else:
            print("Cannot create Spring")
            return 0
        for i in lstmot:
            match i:
                case Point():
                    mot = i
                    break
        else:
            print("Cannot create Spring")
            return 0
        spring = Spring(fix, mot)
        self.addPrimitive(spring)
        return 1
    def primitiveOnPoint(self, point: VECPRIM) -> list:
        lst = []
        for i in self.objects:
            if point in i:
                lst.append(i)
        return lst
    def checkCentres(self, vec: VECPRIM):
        for i in self.objects:
            match i:
                case Point() if (i.centre - vec).len() < two * PointSize:
                    return False
        return True