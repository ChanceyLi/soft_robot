"""
    测试文件
"""

import model
import math
import my_serial


# m = model.Model([1, 0, 0, 0, 0, 0], 0.1)
# print(m.J)

s = my_serial.Serial(band=115200)
print(s.get_port())
