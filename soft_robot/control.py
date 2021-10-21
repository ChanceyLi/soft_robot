import my_serial
import model
import math
from time import sleep
"""
    控制六轴机器人以及软体部分的py文件。
"""


class Control(my_serial.SerialsMng):
    def __init__(self, lst):
        super().__init__(lst)
        self.init_gcode = ['G21\n', 'G90\n', 'G94\n', 'G00 A0 B0 C0 D0 X0 Y0 Z0\n']
        self.m = model.Model([0, 0, 0, 0, 0, 0], delta=0.1)

    def send_and_read(self, theta, delta):
        self.m.set_radian(theta, delta)

        theta = []
        for i in range(7):
            theta.append(int(self.m.get_radian() * 180 / math.pi))