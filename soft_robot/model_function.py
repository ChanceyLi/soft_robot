"""
    建模的函数库文件
"""

import numpy as np
import math


I = np.eye(3)


def screw(p):
    ret = np.matrix([[0, -p[2], p[1]],
                     [p[2], 0, -p[0]],
                     [-p[1], p[0], 0]], dtype=float)
    return ret


def rot(omega, theta):
    ret = I + (math.sin(theta)) * omega + (1 - math.cos(theta)) * (omega @ omega)
    return ret


def T(omega, q, theta):
    ret = np.zeros((4, 4), dtype=float)
    ret[:3, :3] = rot(omega, theta)
    ret[:3, 3:] = (I - ret[:3, :3]) @ q
    ret[3:, :] = np.matrix([0, 0, 0, 1])
    return ret


def delta_vector(delta, l, d):
    ret = np.matrix([[0], [0], [0]], dtype=float)
    if delta == 0:
        ret[0] = 0
        ret[2] = l
    else:
        ret[0] = (1 / delta - 1 / 2) * d * (1 - math.cos(delta * l / d))
        ret[2] = (1 / delta - 1 / 2) * d * math.sin(delta * l / d)
    return ret


def distance(x1, x2):
    res = 0
    for i in range(x1.shape[0]):
        if i < 3:
            res += abs(x1[i] - x2[i])
        else:
            res += abs(x1[i] - x2[i]) * 100
    return res
