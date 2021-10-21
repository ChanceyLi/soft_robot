"""
    初始化文件，附带参数
"""

import numpy as np
import model_function


class Parameters:

    omega = [[0, 0, 1], [1, 0, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 0, 1], [0, 1, 0]]
    q = [np.matrix([[0], [0], [242]], dtype=float), np.matrix([[0], [0], [242]], dtype=float),
         np.matrix([[0], [0], [467]], dtype=float), np.matrix([[0], [0], [695.86]], dtype=float),
         np.matrix([[0], [0], [695.86]], dtype=float), np.matrix([[0], [0], [745.86]], dtype=float),
         np.matrix([[0], [0], [745.86]], dtype=float)]

    S = [np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float), np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float),
         np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float), np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float),
         np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float), np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float),
         np.matrix([[0], [0], [0], [0], [0], [0]], dtype=float)]

    M = np.matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 745.86],
                   [0, 0, 0, 1]], dtype=float)

    def __init__(self):
        pass

    def change_q7(self, dv):
        self.q[6] = self.q[6] + dv

    def init_S(self):
        for i in range(7):
            tmp = -model_function.screw(self.omega[i]) @ self.q[i]
            self.S[i][:3] = tmp
            self.S[i][3] = self.omega[i][0]
            self.S[i][4] = self.omega[i][1]
            self.S[i][5] = self.omega[i][2]
