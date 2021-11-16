import my_serial
import model
import math
from time import sleep
import time
"""
    控制六轴机器人以及软体部分的py文件。
"""


def str_pos(pos_num):
    if pos_num <= 500:
        pos_num = 500
    elif pos_num >= 2500:
        pos_num = 2500

    new_str = "#1P" + str(pos_num) + "T100"
    return new_str


class Control(my_serial.SerialsMny):
    def __init__(self, lst):
        super().__init__(lst)
        self.init_code = ['30', '10', '14']
        self.m = model.Model([0, 0, 0, 0, 0, 0, 0])
        self.__slope = 11.111  # 表示软体部分角度与控制舵机角度数值间的比值
        self.__gap = 1500  # 表示软体部分角度与控制舵机角度数值间的差值

    def send(self, theta):
        """
        :param theta 六轴机器人角度加软体部分的变量
        :提供控制量并将机器人移动到目标位置
        """
        self.m.set_calculation(theta)
        g_code = self.__get_g_code()
        print(g_code)

        self.send_many(0, g_code)
        if len(self.ser_arr) >= 2:
            self.send_many(1, str_pos(self.__gap + int(self.__slope * self.m.get_angel()[6])))

    def auto_run(self, thetas):
        """
        :param thetas 一系列的角度信息
        :自动运行一系列的位置移动
        """
        print("---开始初始化---")
        for code in self.init_code:
            self.ser_arr[0].write(bytes.fromhex(code))
            sleep(1)
            print(code)
        for code in self.init_code:
            self.ser_arr[0].write(bytes.fromhex(code))
            sleep(1)
            print(code)
        print("---初始化完成---")

        idx = 0
        while True:
            time_start = time.time()
            if idx < len(thetas):
                self.send(thetas[idx])
                idx += 1
            while True:
                time_end = time.time()
                data = self.read_many(0)
                if data == '%' or time_end - time_start > 5:
                    print(data)
                    break
                else:
                    sleep(1)
            if idx == len(thetas):
                break

        self.m.set_radians([0, 0, 0, 0, 0, 0, 0.1])
        g_code = self.__get_g_code()
        print(g_code)

        self.send_many(0, g_code)
        if len(self.ser_arr) >= 2:
            self.send_many(1, str_pos(self.__gap + int(self.__slope * self.m.get_angel()[6])))

    def __get_g_code(self):
        g_code = 'G00 J1=' + str(self.m.get_angel()[0]) + ' J2=' + str(self.m.get_angel()[1]) + ' J3=' + \
                 str(self.m.get_angel()[2]) + \
                 ' J4=' + str(self.m.get_angel()[3]) + ' J5=' + str(self.m.get_angel()[4]) + ' J6=' + \
                 str(self.m.get_angel()[5]) + \
                 ' F2000\r\n'
        return g_code
