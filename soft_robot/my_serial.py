"""
    串口通信文件
"""

import serial
import serial.tools.list_ports


class Serial:
    def __init__(self, band=115200, check='无校验位', data=8, stop=1):
        self.port = None
        self.port_list = list(serial.tools.list_ports.comports())
        assert (len(self.port_list) != 0), '无可用串口'
        self.bandRate = band
        self.checkbit = check
        self.databit = data
        self.stopbit = stop

        self.read_data = None
        self.write_data = None

    def show_port(self):
        for i in range(len(self.port_list)):
            print(self.port_list[i])

    def get_port(self):
        return self.port_list

    def open_port(self, port):
        self.port = serial.Serial(port, self.bandRate, timeout=None)

    def close_port(self):
        if self.port != None:
            self.port.close()
            print('串口关闭')

    def read_data_func(self):
        self.read_data = self.port.read(self.port.in_waiting)
        return self.read_data.decode('utf-8')

    def write_data_func(self, data):
        if not self.port.isOpen():
            print('串口未打开')
        else:
            self.port.write(data.encode('utf-8'))


