from enum import Enum

class UpdateModes(Enum):
    Stay = 0
    Static = 1
    Mobile = 2
    Pendulum = 3
    Spring = 4

class RunModes(Enum):
    run = 0
    pause = 1
