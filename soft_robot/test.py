"""
    测试文件
"""

import model
import math
import my_serial


# m = model.Model([1, 0, 0, 0, 0, 0], 0.1)
# print(m.J)

m = model.Model([0.5, 0.3, 1, 0.9, 0.1235, 0.153, 0], delta=0)
# print(m.get_angel())
# print(m.Te)
theta, _ = model.inverse_kinematic(m.Te, [0, 0, 0, 0, 0, 0])
# for j in range(len(theta)):
#     for i in range(theta[j].shape[0]):
#         theta[j][i] = theta[j][i] * 180 / math.pi
print(theta)


