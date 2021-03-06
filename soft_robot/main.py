# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import my_serial
import init
import model
import model_function
import frame_data
import math
import control
import serial
from time import sleep
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # client socket
    # ctl = control.Control(lst=['COM5'])
    # thetas = [[1.73, 0, 0, 0, 0, 0, 0.5], [0, 0, 0, 0, 0, 0, 0.5]]
    # ctl.auto_run(thetas)
    m = model.Model([0.1, 0.02, 0.1, 0.4, 0.2, 0.4, 0])
    print(m.get_position())
    # print(m.J)
    # m.J[:, :6] = m.jacobi_DH()
    # print(m.jacobi())
    xs = np.matrix([[0], [0], [795.86], [0], [0], [0.8]], dtype=float)
    x0 = m.get_position().copy()
    v_old = np.zeros((7, 1), dtype=float)
    M = 21  # 套娃的次数
    N = 5  # 迭代的次数
    T = 1
    alpha = 1e-2
    Xs = []
    Yd = []
    Ys = []
    X_get = []
    Ys.append(x0)
    X_get.append(x0)
    for j in range(M + 1):
        tmp = np.zeros((6, 1), dtype=float)
        for k in range(6):
            tmp[k, 0] = (xs[k] - x0[k]) * j / M + x0[k]
        Yd.append(tmp)
    for i in range(M):
        tmp = np.zeros((6, 1), dtype=float)
        for j in range(6):
            tmp[j, 0] = (xs[j] - x0[j]) * (i+1) / M + x0[j]
        Xs.append(tmp)
    for i in range(M):
        Yd_tmp = np.zeros((6 * N + 6, 1), dtype=float)
        for j in range(N + 1):
            for k in range(6):
                Yd_tmp[6 * j + k, 0] = (Xs[i][k, 0] - x0[k]) * j / N + x0[k]
        # print(Yd)
        nu = m.get_radian().copy()
        u, X_get_tmp = m.model_opt(v_old, Yd_tmp[6:6*N+6, ], N, T, alpha)
        X_get.append(X_get_tmp[6*N-6:, ])

        for j in range(1, N):
            u[j*7:7*j+7] += u[j*7-7:7*j]
        for j in range(N):
            for k in range(7):
                nu[k] += u[7 * j + k]*T
            # print(nu)
        m.set_calculation(nu)
        Ys.append(m.get_position().copy())
        # print(n.get_position())
        x0 = m.get_position().copy()
        # if model_function.distance(Ys[i*6:i*6:12, ], Yd[i*6+6:i*6+12:, ]) > 100:  # This maybe work
        #     i -= 1

    count = 0
    while model_function.distance(Ys[-1], xs) > .1 and count < 5:
        Yd_tmp = np.zeros((6 * N + 6, 1), dtype=float)
        for j in range(N + 1):
            for k in range(6):
                Yd_tmp[6 * j + k, 0] = xs[k, 0]
        # print(Yd)
        nu = m.get_radian().copy()
        u, X_get_tmp = m.model_opt(v_old, Yd_tmp[6:6 * N + 6, ], N, T, alpha)
        X_get.append(X_get_tmp[6 * N - 6:, ])

        for j in range(1, N):
            u[j * 7:7 * j + 7] += u[j * 7 - 7:7 * j]
        for j in range(N):
            for k in range(7):
                nu[k] += u[7 * j + k] * T
            # print(nu)
        m.set_calculation(nu)
        Ys.append(m.get_position().copy())
        Yd.append(xs)
        # print(n.get_position())
        x0 = m.get_position().copy()
        count += 1
    for i in range(6):
        x = []
        y1 = []
        y2 = []
        y3 = []
        for j in range(len(Yd)):
            x.append(j)
            y1.append(Yd[j][i, 0])
            y2.append(Ys[j][i, 0])
            y3.append(X_get[j][i, 0])
        plt.title('Change in the ' + str(i+1) + 'th dim')
        plt.xlabel('iteration time')
        plt.ylabel('position(mm)')
        plt.plot(x, y1, 'b', label='expectation', marker='o', markerfacecolor='black')
        plt.plot(x, y2, 'r', label='real-fact', marker='*')
        # plt.plot(x, y3, 'g', label='opt-fact')
        plt.legend()
        plt.show()


