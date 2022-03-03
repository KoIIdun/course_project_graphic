import numpy as np

flt = np.float32
NULL = np.array([0, 0])
PRIM = int | np.int32 | flt | float
PFLOAT = flt
COLOR = tuple[int, int, int]
SHAPE = tuple
SHAPEI = SHAPE[int, int]
STANDART_WINDOW_SHAPE = (960, 600)
AQUA_COLOR = (16, 44, 58)
SEA_COLOR =	(118,182,196)
SEMI_WHITE_COLOR = 	(222,243,246)
ZERO = flt(0)

def randomColor():

    return [np.random.randint(0, 256) for i in range(3)].copy()