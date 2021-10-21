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



class SerialsMng():
    # list=[name,bps,pixStyle,width,  name,bps,pixStyle,width, ]
    # ["COM3",250000,0x13, 45, "COM4",250000,0x13, 45, ]
    def __init__(self, lst):

        self.ser_count = len(lst)
        self.ser_arr = []
        for i in range(self.ser_count):
            sop = SerialOP(lst[i][0], lst[i][1], lst[i][2], lst[i][3])
            print(lst[i][0], lst[i][1], lst[i][2], lst[i][3])
            self.ser_arr.append(sop)
        print(self.ser_arr)

    def setdata(self, idx, data):
        sop = self.ser_arr[idx]
        sop.d.setDataToArray(data)

    def setdataAndsend(self, idx, data):

        sop = self.ser_arr[idx]
        # print("xxxxxxxxxxxxxxxxxxx")
        # print( data)
        # print(sop.d.pkgLen)
        if not sop.busy:
            sop.d.setDataToArray(data)
            # print(sop.d.buf)
            sop.serialSend()

    def splitData(self, data, sidx=0, eidx=0):
        b = eidx <= len(data) and sidx >= 0 and eidx - sidx > 0
        if (b):
            d = data[sidx:eidx]
            return b, d
        return False, []

    def sendFrameData(self, data, pixstyle=4, width=45, height=15):
        datlen = len(self.ser_arr)
        # 每个串口设备控制一行
        c = 0
        u = pixstyle * width
        # print(u)
        for x in range(datlen):  #

            # 获取切割的数据，发送到对应的节点
            b, d = self.splitData(data, c * u, (c + 1) * u)
            # print(c, c * u, (c + 1) * u,b,d)
            if b:
                # 串口发送1

                self.setdataAndsend(x, d)
            c += 1

    def sendFrameData_splited(self, data):
        dvclen = len(self.ser_arr)

        datlen = len(data)
        # 每个串口设备控制 data 一个子数组的数据
        if dvclen < datlen:
            datlen = dvclen
        # print(u)
        for x in range(datlen):  #
            self.setdataAndsend(x, data[x])





class SerialOP():
    no_error = True

    def __init__(self, serialPort, baudRate, pixStyle, width=45):
        # 打开串口
        # serialPort = "COM3"  # 串口
        # baudRate =250000  # 波特率
        self.serialPort = serialPort
        self.baudRate = baudRate
        self.createSer()
        self.d = FrameData(pixStyle, width, 1)
        t = threading.Thread(target=self.thread_autoReCreat)  # 自动连接串口线程
        t.daemon = True
        t.start()

    def createSer(self):
        try:
            self.ser = serial.Serial(self.serialPort, self.baudRate, timeout=0)  # !!!!!!!!!!!!!!!!!!!!!!!!!!无阻塞
            self.no_error = True
            print("参数设置：串口=%s ，波特率=%d" % (self.serialPort, self.baudRate))
            self.busy = False
        except:
            self.no_error = False
            print("ERROE:参数设置：串口=%s ，波特率=%d" % (self.serialPort, self.baudRate))

    def thread_autoReCreat(self):
        while 1:
            if (not self.no_error):
                print("serail err relinking..")
                try:
                    self.createSer()
                except:
                    pass
            time.sleep(1)

    def serialSendData(self, dat):
        datlen = len(dat)
        packstyle = str(datlen) + 'B'  # B 0-255
        req = struct.pack(packstyle, *dat)
        if hasattr(self, 'ser'):
            try:
                self.ser.write(req)
            except serial.serialutil.SerialException:
                self.no_error = False

    def serialSend(self):
        if not self.busy:
            if hasattr(self, 'ser'):
                # print("serialSend")
                try:
                    self.busy = True
                    self.ser.write(self.d.packBytes())
                    self.busy = False
                    # print( self.ser.readline())#read会阻塞
                except serial.serialutil.SerialException:
                    self.no_error = False

    def testCreateDataToSend(self):
        # print("init", d.buf)
        # serialSendData(d.buf)
        slp = 1
        self.d.setDataToOn()
        print("on", self.d.buf)
        self.serialSendData(self.d.buf)
        time.sleep(slp)
        self.d.setDataToRGBW(255)
        print("r", self.d.buf)
        self.serialSendData(self.d.buf)
        time.sleep(slp)
        self.d.setDataToRGBW(0, 255)
        print("g", self.d.buf)
        self.serialSendData(self.d.buf)
        time.sleep(slp)
        self.d.setDataToRGBW(0, 0, 255)
        print("b", self.d.buf)
        self.serialSendData(self.d.buf)
        time.sleep(slp)
        # d.setDataToRGBW(0, 0, 0, 255)
        # print("w", d.buf)
        # serialSendData(d.buf)
        # d.setDataToRGBW(255, 255, 255, 255)
        # print("rgbw", d.buf)
        # serialSendData(d.buf)

    def thread_ssend(self):
        # 收发数据
        c = 0
        while 1:

            print(c)
            self.testCreateDataToSend()

            if (not self.no_error):
                print("serail err relinking..")
                try:
                    self.createSer()
                except:
                    pass
                time.sleep(1)
            if c > 9999:
                c = 0

    def thread_srecv(self):
        while 1:
            print("recv:\n")
            print(self.ser.readline())  # 可以接收中文


if __name__ == "__main__":
    def main():
        # client socket
        sop = SerialOP("COM3", 250000, 0x13, 45)
        # 分别启动听和说线程
        # t = threading.Thread(target=sop.thread_ssend)  # 注意当元组中只有一个元素的时候需要这样写, 否则会被认为是其原来的类型
        # t.daemon=True
        # t.start()

        # t1 = threading.Thread(target=sop.thread_srecv)
        # t1.daemon=True
        # t1.start()
        import time

        while 1:
            sop.testCreateDataToSend()
            time.sleep(1)


    # main()
    sconfig = [["COM21", 250000, 0x13, 45], ["COM210", 250000, 0x13, 45]]  #
    smng = SerialsMng(sconfig)
    while 1:
        # print(smng.ser_arr[0])
        # smng.setdata(0, [0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0])

        smng.setdataAndsend(0, [0xff, 0, 0, 0xff, 0, 0])
        time.sleep(1)
        smng.setdataAndsend(0, [0, 0xff, 0, 0, 0xff, 0])
        time.sleep(1)
        smng.setdataAndsend(0, [0, 0, 0xff, 0, 0, 0xff])
        time.sleep(1)
        smng.setdataAndsend(0, [0xff, 0xff, 0xff, 0xff, 0xff, 0xff])  # '''
        time.sleep(1)
        print('123')
    # ser.close()