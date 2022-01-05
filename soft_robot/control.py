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
        self.__slope = -11.111*180/math.pi  # 表示软体部分角度与控制舵机角度数值间的比值
        self.__gap = 1500  # 表示软体部分角度与控制舵机角度数值间的差值

    def send_robot(self, serial_num1, theta):
        """发送控制信号控制机械臂
        Args:
            serial_num1 六轴机器人控制信号串口 -> int
            theta 控制角度 -> list len(theta) = 7
        Returns:
            Null
        """
        self.m.set_calculation(theta)
        g_code = self.__get_g_code()
        self.send_many(serial_num1, g_code)

    def send(self, serial_num1, serial_num2, theta):
        """发送控制信号
        Args:
            serial_num1 六轴机器人控制信号串口 -> int
            serial_num2 软体末端控制信号串口 -> int
            theta 控制角度 -> list len(theta) = 7
        Returns:
            Null
        """
        self.send_robot(serial_num1, theta)
        self.send_many(serial_num2, str_pos(self.__gap + int(self.__slope * self.m.get_angel()[6])))

    def receive(self, serial_num, sensor_num):
        """接收力反馈的信号，并进行分割
        Args:
            serial_num 力反馈串口编号 -> str
                'COM4', 'COM5', etc
            sensor_num 力反馈传感器数量 -> int
                sensor_num <= 10
        Returns:
            力反馈信号 -> list len(return) = sensor_num
        """
        message = self.read_many(serial_num)
        l, r = 0, 0
        ret = []
        for i in range(len(message)):
            if message[i] == ',' or message[i] == ';':
                if len(ret) == 0:
                    l = r
                else:
                    l = r + 1
                r = i
                ret.append(int(message[l:r]))
                if len(ret) >= sensor_num:
                    break
        return ret

    def initial(self):
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

    def auto_run(self, serial_num1, serial_num2, serial_num3, thetas):
        """自动执行一系列控制信号
        Args:
            serial_num1 六轴机器人控制信号串口编号 -> int
            serial_num2 软体末端控制信号串口编号 -> int
            serial_num3 力反馈控制信号串口编号 -> int
            thetas 一系列角度 -> list 其中每个theta含有7个角度信息
        Returns:
            Null
        """
        idx = 0
        while True:
            time_start = time.time()
            if idx < len(thetas):
                self.send(serial_num1, serial_num2, thetas[idx])
                idx += 1
            if self.receive(serial_num3, 1)[0] > 500:
                print("我摸到了，返回！")
                break
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

        self.send(serial_num1, serial_num2, [0, 0, 0, 0, 0, 0, 0])

    def __get_g_code(self):
        g_code = 'G00 J1=' + str(self.m.get_angel()[0]) + ' J2=' + str(self.m.get_angel()[1]) + ' J3=' + \
                 str(self.m.get_angel()[2]) + \
                 ' J4=' + str(self.m.get_angel()[3]) + ' J5=' + str(self.m.get_angel()[4]) + ' J6=' + \
                 str(self.m.get_angel()[5]) + \
                 ' F2000\r\n'
        return g_code
