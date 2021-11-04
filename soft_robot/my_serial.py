"""
    串口通信文件
"""
# -*- coding: utf-8 -*-
import serial, time, threading, struct
import serial.tools.list_ports
from frame_data import FrameData
import struct
from ctypes import create_string_buffer

class Serial:
    def __init__(self, port, band=115200, check='无校验位', timeout=2, bytesize=8, stopbits=1):
        self.port = serial.Serial(port, band, timeout=timeout, bytesize=bytesize, stopbits=stopbits)

    def close_port(self):
        if self.port != None:
            self.port.close()
            print('串口关闭')

    def read_data_de(self):
        return self.port.readline().decode()

    def write_data_en(self, data):
        if not self.port.isOpen():
            print('串口未打开')
        else:
            self.port.write(data.encode())

    def read(self):
        return self.port.read()

    def write(self, data):
        if not self.port.isOpen():
            print('串口未打开')
        else:
            self.port.write(data)


class SerialsMny():
    def __init__(self, lst):

        self.ser_count = len(lst)
        self.ser_arr = []
        for i in range(self.ser_count):
            sop = Serial(lst[i])
            print(lst[i])
            self.ser_arr.append(sop)
        print(self.ser_arr)

    def send_many(self, idx, data):

        sop = self.ser_arr[idx]
        sop.write_data_en(data)

    def read_many(self, idx):
        return self.ser_arr[idx].read_data_de()



