# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import my_serial
import init
import model
import model_function
import frame_data
import math
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # client socket
    sop = my_serial.SerialsMng([["COM4", 115200, 0x13, 45]])
    init_gcode = ['G21\n', 'G90\n', 'G94\n', 'G00 A0 B0 C0 D0 X0 Y0 Z0\n']
    m = model.Model([0, 0, 0, 0, 0, 0], delta=0.1)
    theta = []
    for i in range(7):
        theta.append(int(m.get_radian() * 180 / math.pi))
    run_gcode = 'G01 A' + str(theta[0]) + ' B' + str(theta[1]) + ' C' + str(theta[2]) + ' D' + str(theta[3]) + ' X' + \
        str(theta[4]) + ' F2000\n'
    run_gcode6 = 'M13.' + str(theta[5]).rjust(3, '0') + '\n'

    while True:
        print('舵机范围500-2500')
        pos1 = input("输入舵机的位置：")
        pos1_num = int(pos1)
        if pos1_num == -1:
            break
        if pos1_num <= 500:
            pos1_num = 500
        elif pos1_num >= 2500:
            pos1_num = 2500

        newstr = "#1P" + str(pos1_num) + "T100"
        sop.setdataAndsend(0, newstr.encode("gbk"))

    # 分别启动听和说线程
    # t = threading.Thread(target=sop.thread_ssend)  # 注意当元组中只有一个元素的时候需要这样写, 否则会被认为是其原来的类型
    # t.daemon=True
    # t.start()

    # t1 = threading.Thread(target=sop.thread_srecv)
    # t1.daemon=True
    # t1.start()
    # my_serial.run_pos()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
