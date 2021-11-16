"""
    测试文件
"""

import model
import math
import my_serial
import random
import action
import numpy as np

# m = model.Model([1, 0, 0, 0, 0, 0], 0.1)
# print(m.J)

a = action.Action()
position = np.matrix([[150], [170], [200], [math.pi], [0], [0]], dtype=float)
theta = a.set_radians_with_push_mode(position, theta_soft=0.1)
print(theta)
a.controller.m.set_calculation(theta)
pos = model.trans_matrix2position(a.controller.m.Te)
print(pos)
print(a.controller.m.get_position())





