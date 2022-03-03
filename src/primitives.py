import pyglet.image
from pyglet import *
import numpy as np
from pyglet.gl import *
from constants import *

def makeCircleCoords(radius: int, points: int) -> np.ndarray:
    theta = np.linspace(0, 2 * np.pi, points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return np.stack((x, y), axis=1)
def delete(id, dict):
    pp = []
    for i in dict.items():
        if i[1] == id:
            pp.append(i[0])
    for i in pp:
        dict.pop(i)
    return dict

#pyglet.graphics.vertex_list()
class TextureLoader:
    try:
        springraw = pyglet.image.load(r"C:\Users\Kolldun\PycharmProjects\course_project_physic\textures\spring.png")
    except:
        try:
            springraw = pyglet.image.load(os.getcwd() +r"\spring.png")
        except:
            springraw = None
            print("WARNING: TextureLoader cant load texture, path " + os.getcwd() + "\spring.png doesnt exists")
            #raise Exception("TextureLoader cant load texture path " + os.getcwd() + "\spring.png doesnt exists")

    tex = springraw.get_texture() if springraw else None
    def __init__(self):
        if TextureLoader.springraw:
            self.tex.tex_coords = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0)
            glBindTexture(self.tex.target, self.tex.id)
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RGBA,
                self.springraw.width,
                self.springraw.height,
                0,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                self.springraw.get_image_data()
                              .get_data("RGBA", self.springraw.width * 4)
            )


            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE)


class Primitives:
    circleLists: dict[PFLOAT, dict[PFLOAT, int]] = dict()
    rectangleLists: dict[tuple[int, int], int] = dict()
    tloader = TextureLoader()
    @staticmethod
    def getCircle(radius : PFLOAT, points: PFLOAT):
        try:
            return Primitives.circleLists[radius][points]
        except:
            id = glGenLists(GL_COMPILE)
            coords = makeCircleCoords(radius, points)
            glNewList(id, GL_COMPILE)
            glBegin(GL_POLYGON)
            for vertex in coords:
                glVertex3f(*vertex, -0.1)
            glEnd()
            glEndList()
            if radius not in Primitives.circleLists.keys():
                Primitives.circleLists[radius] = dict()
            Primitives.circleLists[radius][points] = id
            return id
    @staticmethod
    def delete(id):
        Primitives.circleLists = delete(id, Primitives.circleLists)
        Primitives.rectangleLists = delete(id, Primitives.rectangleLists)
        #glDeleteLists(id)


    @staticmethod
    def getRectangle(length, width):
        tup = (length, width)
        try:
            return Primitives.rectangleLists[tup]
        except:
            id = glGenLists(GL_COMPILE)
            glNewList(id, GL_COMPILE)

            glBegin(GL_QUADS)
            z = -1
            glVertex3f(0, - width / 2, z)
            glTexCoord2d(0, 0)
            glVertex3f(0, width / 2, z)
            glTexCoord2d(0, 1)
            glVertex3f(length, width / 2, z)
            glTexCoord2d(1, 1)
            glVertex3f(length, - width / 2, z)
            glTexCoord2d(1, 0)

            glEnd()
            glEndList()
            Primitives.rectangleLists[tup] = id
            return id