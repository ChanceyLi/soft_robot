"""
    测试文件
"""

import model
import math
import my_serial
import random
import action
import numpy as np
import control
import time
import accelerate

# m = model.Model([1, 0, 0, 0, 0, 0], 0.1)
# print(m.J)
# Action 文件的测试
# a = action.Action(['COM3', 'COM5', 'COM6'])
# a.controller.send_many(1, control.str_pos(1500))
# position = np.matrix([[0], [400], [100], [math.pi], [0], [0]], dtype=float)
# theta = a.move_and_push(position, theta_soft=1.0)

# Control 文件 力反馈测试
# c = control.Control(['COM10'])
# count = 0
# l = []
# while count < 1000:
#     n = c.receive(0, 1)
#     l.append(n)
#     print(n)
#     count += 1
#     time.sleep(0.1)
# video = r'mv-without-touch.mp4'
# result = r'mv-without-touch-accelerate.mp4'
# accelerate.accelerate_video(video, result, A=2)
m = model.Model([0, 0, 0, 0, 0, 0, 0])
print(m.get_position())
c = control.Control(['COM3'])
positions = []
position = np.array([[0], [300], [300], [math.pi], [0], [0]])
positions.append(position)


def get_angles(pos):
    T = model.position2trans_matrix(pos)
    thetas = [0, 0, 0, 0, 0, 0, 0]
    tmp, _ = model.inverse_kinematic(T, [0, 0, 0, 0, 0, 0])
    for i in range(6):
        thetas[i] = tmp[i]
    m.set_calculation(thetas)
    print(thetas)
    return thetas


thetass = []
for position in positions:
    thetass.append(get_angles(position))

c.initial()


def run(thetas):
    idx = 0
    while True:
        time_start = time.time()
        if idx < len(thetas):
            c.send_robot(0, thetas[idx])
            idx += 1
        while True:
            time_end = time.time()
            data = c.read_many(0)
            if data == '%' or time_end - time_start > 5:
                print(data)
                break
            else:
                time.sleep(1)
        if idx == len(thetas):
            break


run(thetass)
time.sleep(10)
thetass = []
positions = []
position = np.array([[0], [450], [170], [math.pi], [0], [0]])
positions.append(position)
position = np.array([[0], [450], [155], [math.pi], [0], [0]])
positions.append(position)
for position in positions:
    thetass.append(get_angles(position))
run(thetass)

time.sleep(5)
thetass = []
positions = []
position = np.array([[0], [450], [250], [math.pi], [0], [0]])
positions.append(position)
position = np.array([[200], [200], [250], [math.pi], [0], [0]])
positions.append(position)

for position in positions:
    thetass.append(get_angles(position))
run(thetass)
time.sleep(20)
thetass = []
positions = []
position = np.array([[100], [320], [250], [math.pi], [0], [0]])
positions.append(position)

for position in positions:
    thetass.append(get_angles(position))
run(thetass)
time.sleep(20)
thetass = []
positions = []
position = np.array([[250], [200], [250], [math.pi], [0], [0]])
positions.append(position)

for position in positions:
    thetass.append(get_angles(position))
run(thetass)
time.sleep(20)
c.send_robot(0, [0, 0, 0, 0, 0, 0, 0])

