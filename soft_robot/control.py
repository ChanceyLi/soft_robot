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
        """
        :param theta 六轴机器人角度 delta 软体部分的变量
        :提供控制量并将机器人移动到目标位置
        """
        self.m.set_radian(theta, delta)

        theta = []
        for i in range(7):
            theta.append(int(self.m.get_radian() * 180 / math.pi))
        run_gcode = 'G01 A' + str(theta[0]) + ' B' + str(theta[1]) + ' C' + str(theta[2]) + ' D' + str(
            theta[3]) + ' X' + \
                    str(theta[4]) + ' F2000\n'
        run_gcode6 = 'M13.' + str(theta[5]).rjust(3, '0') + '\n'
        self.setdataAndsend(0, run_gcode)
        self.setdataAndsend(1, run_gcode6)
        return self.read_data()

    def auto_run(self, thetas, deltas):
        """
        :param thetas, deltas 一系列的角度信息
        :自动运行一系列的位置移动
        """
        for theta, delta in thetas, deltas:
            res = self.send_and_read(theta, delta)
            if res[0] == '%':
                continue
            else:
                pass
            sleep(1)
