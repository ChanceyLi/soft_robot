"""
    建模文件
"""

import init
import model_function
import numpy as np
import math
import cvxopt

# 原六轴机器人的相关参数以及约束条件，待修改成最新的六轴机器人与软体部分的参数以及约束
# a1, a2, a3, b1, b4, b6 = 32, 108, 20, 80, 176, 20  # 机器人DH参数
qMax = np.matrix([[180], [115], [130], [180], [165], [180], [90]], dtype=float)  # 机器人角度限制
qMin = -qMax
vMax = np.matrix([[200], [200], [200], [200], [200], [200], [200]], dtype=float)  # 机器人关节速度限制
vMin = -vMax
qMax_r = qMax * math.pi / 180
qMin_r = qMin * math.pi / 180
vMax_r = vMax * math.pi / 180
vMin_r = vMin * math.pi / 180
# x0, y0, z0, z1 = 200, -100, 20, 40  # 表示像素点(0, 0)的对应世界坐标位置
x_min, x_max, y_min, y_max, z_min, z_max = -50, 250, -150, 150, 0, 50  # 表示机器人末端的工作范围、约束


class Model(init.Parameters):
    __theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    __Theta = [0, 0, 0, 0, 0, 0, 0]
    __d = 10  # soft body distance
    __l = 50  # soft body length
    delta = 0.1  # soft body length change rate
    T = np.matrix([4, 4], dtype=float)
    Te = np.matrix([4, 4], dtype=float)
    T_all = []
    __pos = np.zeros((6, 1), dtype=float)

    def __init__(self, theta, delta=0):
        super().__init__()
        self.delta = delta
        self.__theta[6] = self.delta * self.__l / self.__d
        self.__theta = theta
        self.change_q7(model_function.delta_vector(self.delta, self.__l, self.__d))
        self.init_S()
        for i in range(7):
            self.__Theta[i] = int(self.__theta[i] * 180 / math.pi)
        self.J = np.zeros((6, len(self.omega)), dtype=float)
        self.forward_matrix()
        self.jacobi_D()

    def set_radian_delta(self, theta, delta):
        """
        :param theta 前六轴的角度 delta 软体部分的变量
        :更新模型中的角度[Theta1, Theta2, ..., Theta7]
        """
        self.__theta[:6] = theta
        self.__solve_delta(delta)
        for i in range(6):
            self.__Theta[i] = int(self.__theta[i] * 180 / math.pi)

    def set_radians(self, theta):
        """
        :param theta 前六轴加软体部分的角度，共七个。
        :更新模型角度
        """
        self.__theta = theta
        self.delta = self.__theta[6] * self.__d / self.__l
        for i in range(7):
            self.__Theta[i] = int(self.__theta[i] * 180 / math.pi)
        self.change_q7(model_function.delta_vector(self.delta, self.__l, self.__d))
        self.init_S()

    def set_calculation(self, theta):
        self.set_radians(theta)
        self.forward_matrix()
        self.jacobi_D()

    def __solve_delta(self, delta):
        self.delta = delta
        self.__theta[6] = self.delta * self.__l / self.__d
        self.__Theta[6] = int(self.__theta[6] * 180 / math.pi)

    def get_radian(self):
        """
        :param
        :获取模型中的角度[theta1, theta2, ..., theta7]
        """
        return self.__theta

    def get_angel(self):
        """
        :param
        :获取模型中的角度[Theta1, Theta2, ..., Theta7]
        """
        return self.__Theta

    def get_position(self):
        """
        :param
        :获取模型中的末端执行器位置
        """
        return self.__pos

    def forward_matrix(self):
        """
        :param
        :前向齐次转移矩阵
        """
        T = []
        for i in range(len(self.omega)):
            T.append(model_function.T(model_function.screw(self.omega[i]), self.q[i], self.__theta[i]))

        Te_soft = T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ T[5] @ T[6] @ self.M_all[6]
        Te = T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ T[5] @ self.M_all[5]
        self.Te = Te
        self.T_all = T
        self.T = Te_soft
        beta = math.atan2(-self.T[2, 0], math.sqrt(self.T[0, 0] ** 2 + self.T[1, 0] ** 2))
        if abs(beta - math.pi / 2) < 0.00001:
            alpha = math.atan2(self.T[0, 1], self.T[1, 1])
            gamma = 0
        elif abs(beta + math.pi / 2) < 0.00001:
            alpha = -math.atan2(self.T[0, 1], self.T[1, 1])
            gamma = 0
        else:
            alpha = math.atan2(self.T[2, 1] / math.cos(beta), self.T[2, 2] / math.cos(beta))
            gamma = math.atan2(self.T[1, 0] / math.cos(beta), self.T[0, 0] / math.cos(beta))
        self.__pos[:3, 0] = self.T[:3, 3]
        self.__pos[3, 0] = alpha
        self.__pos[4, 0] = beta
        self.__pos[5, 0] = gamma
        # return Te

    def jacobi(self):
        """
        :param
        :雅可比矩阵
        """
        J = np.zeros((6, len(self.__theta)), dtype=float)
        for i in range(len(self.__theta)):
            tmp = np.eye(4)
            for j in range(i):
                tmp = tmp @ self.T_all[j]
            ttmp = np.zeros((6, 6), dtype=float)
            ttmp[:3, :3] = tmp[:3, :3]
            ttmp[3:, 3:] = tmp[:3, :3]
            ttmp[3:, :3] = model_function.screw([tmp[0, 3], tmp[1, 3], tmp[2, 3]]) @ tmp[:3, :3]
            J[:, i:i + 1] = ttmp @ self.S[i]
        # for i in range(len(self.theta)):
        #     Tmp = np.eye(4)
        #     for j in range(i+1):
        #         Tmp = Tmp @ self.T_all[j]
        #     Tmp = Tmp @ self.M_all[i]
        #     p = Tmp[:3, 3]
        #     J[:3, i] = model_function.screw(Tmp[:3, 2]) @ (self.pos[:3, 0] - p)
        #     J[3:, i] = Tmp[:3, 2]
        # self.J = J
        return J

    def jacobi_D(self):
        delta = 0.001
        th = self.__theta.copy()
        pos = self.__pos.copy()
        J = np.zeros((6, len(self.__theta)), dtype=float)
        for i in range(len(self.__theta)):
            tmp_theta = th.copy()
            tmp_theta[i] += delta
            self.set_radians(tmp_theta)
            pos1 = self.__pos.copy()
            tmp_theta = th.copy()
            tmp_theta[i] -= delta
            self.set_radians(tmp_theta)
            pos2 = self.__pos.copy()
            J[:, i:i + 1] = (pos1 - pos2) / (2 * delta)
        self.set_radians(th)
        self.forward_matrix()
        self.J = J
        return J

    def jacobi_DH(self):
        my_theta = [0, -math.pi / 2, math.pi / 2, 0, 0, 0]
        sl = [1, 1, 1, 1, -1, 1]
        for i in range(6):
            my_theta[i] += sl[i] * self.__theta[i]
        L1, L2, L3, L4 = 242.0, 225.0, 228.86, 50
        c = [0, 0, 0, 0, 0, 0]
        s = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            c[i], s[i] = math.cos(my_theta[i]), math.sin(my_theta[i])
        T01 = np.matrix([[c[0], -s[0], 0, 0],
                         [s[0], c[0], 0, 0],
                         [0, 0, 1, L1],
                         [0, 0, 0, 1]], dtype=float)
        T12 = np.matrix([[c[1], -s[1], 0, 0],
                         [0, 0, 1, 0],
                         [-s[1], -c[1], 0, 0],
                         [0, 0, 0, 1]], dtype=float)
        T23 = np.matrix([[c[2], -s[2], 0, L2],
                         [s[2], c[2], 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]], dtype=float)
        T34 = np.matrix([[c[3], -s[3], 0, 0],
                         [0, 0, -1, -L3],
                         [s[3], c[3], 0, 0],
                         [0, 0, 0, 1]], dtype=float)
        T45 = np.matrix([[c[4], -s[4], 0, 0],
                         [0, 0, -1, 0],
                         [s[4], c[4], 0, 0],
                         [0, 0, 0, 1]], dtype=float)
        T56 = np.matrix([[c[5], -s[5], 0, 0],
                         [0, 0, 1, L4],
                         [-s[5], -c[5], 0, 0],
                         [0, 0, 0, 1]], dtype=float)
        T46 = T45 @ T56
        T36 = T34 @ T46
        T26 = T23 @ T36
        T16 = T12 @ T26
        T06 = T01 @ T16
        J = np.zeros((6, 6), dtype=float)
        J[3:, 0:1] = T06[2, :3].T
        J[3:, 1:2] = T16[2, :3].T
        J[3:, 2:3] = T26[2, :3].T
        J[3:, 3:4] = T36[2, :3].T
        J[3:, 4:5] = T46[2, :3].T
        J[3:, 5:6] = T56[2, :3].T
        for i in range(3):
            J[i, 0] = T06[0, 3] * T06[1, i] - T06[1, 3] * T06[0, i]
            J[i, 1] = T16[0, 3] * T16[1, i] - T16[1, 3] * T16[0, i]
            J[i, 2] = T26[0, 3] * T26[1, i] - T26[1, 3] * T26[0, i]
            J[i, 3] = T36[0, 3] * T36[1, i] - T36[1, 3] * T36[0, i]
            J[i, 4] = T46[0, 3] * T46[1, i] - T46[1, 3] * T46[0, i]
            J[i, 5] = T56[0, 3] * T56[1, i] - T56[1, 3] * T56[0, i]
        return J

    def model_opt(self, v_old, Yd, N=50, T=0.1, alpha=0.1):
        """
        :param v_old    (dim, 1)    表示上一次控制作用后的速度
        :      Yd       (6 * N, 1)表示期望路径
        :      N = 50   起始点到目标点 点的个数
        :      T = 0.1  v = (x(i+1) - x(i)) / T
        :利用cvx求解优化问题
        """
        x0 = self.__pos.copy()
        dim = len(self.__theta)

        q0 = np.zeros((dim, 1), dtype=float)
        for i in range(dim):
            q0[i, 0] = self.__theta[i]
        B = np.zeros((6 * N, dim * N), dtype=float)
        I_N = np.zeros((dim * N, dim * N), dtype=float)
        I = np.eye(dim * N, dim * N)
        for i in range(N):
            for j in range(N):
                if i >= j:
                    B[6 * i:6 * i + 6, dim * j:dim * j + dim] = T * self.J
                    I_N[dim * i:dim * i + dim, dim * j:dim * j + dim] = np.eye(dim)

        VMin = np.zeros((dim * N, 1), dtype=float)
        VMax = np.zeros((dim * N, 1), dtype=float)
        QMin = np.zeros((dim * N, 1), dtype=float)
        QMax = np.zeros((dim * N, 1), dtype=float)
        for i in range(N):
            VMin[dim * i: dim * i + dim, ] = vMin_r - v_old
            VMax[dim * i: dim * i + dim, ] = vMax_r - v_old
            QMin[dim * i: dim * i + dim, ] = (qMin_r - q0 - T * v_old) / T
            QMax[dim * i: dim * i + dim, ] = (qMax_r - q0 - T * v_old) / T

        X0 = np.zeros((6 * N, 1), dtype=float)
        V_old = np.zeros((dim * N, 1), dtype=float)
        for i in range(N):
            X0[6 * i: 6 * i + 6, ] = x0
            V_old[dim * i: dim * i + dim, ] = v_old

        G = np.zeros((4 * dim * N, dim * N), dtype=float)
        B_ = np.zeros((4 * dim * N, 1), dtype=float)
        for i in range(4):
            if i == 0:
                G[dim * N * i: dim * N * i + dim * N, ] = -I_N
                B_[dim * N * i: dim * N * i + dim * N, ] = -VMin
            elif i == 1:
                G[dim * N * i: dim * N * i + dim * N, ] = I_N
                B_[dim * N * i: dim * N * i + dim * N, ] = VMax
            elif i == 2:
                G[dim * N * i: dim * N * i + dim * N, ] = -I_N @ I_N
                B_[dim * N * i: dim * N * i + dim * N, ] = -QMin
            elif i == 3:
                G[dim * N * i: dim * N * i + dim * N, ] = I_N @ I_N
                B_[dim * N * i: dim * N * i + dim * N, ] = QMax

        Hess = ((B @ I_N).T @ (B @ I_N)) + alpha * I
        c = (X0 - Yd + B @ V_old).T @ B @ I_N
        u = cvxopt_solve_qp(Hess, c.T, G, B_, None, None)
        X_get = X0 + B @ I_N @ u
        # print(X_get[6*N-6:, ])
        # print(u)
        return u, X_get


def func_opt(H, c):
    v = lambda x: 1 / 2 * x.T @ H @ x + c @ x
    return v


def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [cvxopt.matrix(P), cvxopt.matrix(q)]
    if G is not None:
        args.extend([cvxopt.matrix(G), cvxopt.matrix(h)])
        if A is not None:
            args.extend([cvxopt.matrix(A), cvxopt.matrix(b)])
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x'])


def inverse_kinematic(T, initial_theta):
    L1, L2, L3, L4 = 242.0, 225.0, 228.86, 50
    theta = [np.zeros((6, 1), dtype=float)]
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    ox, oy, oz = T[0, 1], T[1, 1], T[2, 1]
    ax, ay, az = T[0, 2], T[1, 2], T[2, 2]
    theta[0][0] = math.atan2(py - L4 * ay, px - L4 * ax)
    r12 = math.cos(theta[0][0]) * ox + math.sin(theta[0][0]) * oy
    r22 = -math.sin(theta[0][0]) * ox + math.cos(theta[0][0]) * oy
    r32 = oz
    r13 = math.cos(theta[0][0]) * ax + math.sin(theta[0][0]) * ay
    r23 = -math.sin(theta[0][0]) * ax + math.cos(theta[0][0]) * ay
    r33 = az
    qx = math.cos(theta[0][0]) * px + math.sin(theta[0][0]) * py
    qy = -math.sin(theta[0][0]) * px + math.cos(theta[0][0]) * py
    qz = pz - L1
    k_tmp = ((qx - r13 * L4) ** 2 + (qz - r33 * L4) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2)
    if L3**2 < k_tmp**2:
        return initial_theta, 0
    theta[0][2] = -math.atan2(k_tmp, math.sqrt(L3 ** 2 - k_tmp ** 2))
    theta[0][1] = math.atan2((qx - r13 * L4) * (L2 * math.sin(theta[0][2]) - L3) -
                             (qz - r33 * L4) * (L2 * math.cos(theta[0][2])),
                             (qx - r13 * L4) * (L2 * math.cos(theta[0][2])) +
                             (qz - r33 * L4) * (L2 * math.sin(theta[0][2]) - L3)) - theta[0][2]

    if theta[0][1] > math.pi:
        theta[0][1] = theta[0][1] - 2 * math.pi
    if theta[0][1] < -math.pi:
        theta[0][1] = theta[0][1] + 2 * math.pi
    DOUBLE_ABS_MIN = 1e-10
    if abs(r23) < DOUBLE_ABS_MIN and abs(r33 * math.sin(theta[0][1] + theta[0][2]) - r13 * math.cos(theta[0][1] +
                                         theta[0][2])) < DOUBLE_ABS_MIN:
        theta[0][4] = 0
        theta[0][3] = 0
    else:
        theta[0][3] = math.atan2(r23, r33 * math.sin(theta[0][1] + theta[0][2]) -
                                 r13 * math.cos(theta[0][1] + theta[0][2]))
        if abs(math.sin(theta[0][3])) < 0.01:
            theta[0][4] = math.atan2((r33 * math.sin(theta[0][1] + theta[0][2]) -
                                      r13 * math.cos(theta[0][1] + theta[0][2])) / math.cos(theta[0][3]),
                                     -r13 * math.sin(theta[0][1] + theta[0][2]) -
                                     r33 * math.cos((theta[0][1] + theta[0][2])))
        else:
            theta[0][4] = math.atan2(r23 / math.sin(theta[0][3]),
                                     -r13 * math.sin(theta[0][1] + theta[0][2]) -
                                     r33 * math.cos((theta[0][1] + theta[0][2])))
    if abs(math.sin(theta[0][4])) < 0.01:
        theta[0][5] = math.atan2((r22 * math.sin(theta[0][3]) +
                                  math.cos(theta[0][3]) * (r32 * math.sin(theta[0][1] + theta[0][2]) -
                                                           r12 * math.cos(theta[0][1] + theta[0][2]))) /
                                 math.cos(theta[0][4]),
                                 math.sin(theta[0][3]) * (r32 * math.sin(theta[0][1] + theta[0][2]) -
                                                          r12 * math.cos(theta[0][1] + theta[0][2])) -
                                 math.cos(theta[0][3]) * r22)
    else:
        theta[0][5] = math.atan2((r12 * math.sin(theta[0][1] + theta[0][2]) +
                                  r32 * math.cos(theta[0][1] + theta[0][2])) / math.sin(theta[0][4]),
                                 math.sin(theta[0][3]) * (r32 * math.sin(theta[0][1] + theta[0][2]) -
                                                          r12 * math.cos(theta[0][1] + theta[0][2])) -
                                 math.cos(theta[0][3]) * r22)

    # -----
    theta.append(theta[0].copy())

    if theta[1][3] > 0:
        theta[1][3] = theta[1][3] - math.pi
    else:
        theta[1][3] = theta[1][3] + math.pi

    if abs(math.sin(theta[1][3])) < 0.01:
        theta[1][4] = math.atan2((r33 * math.sin(theta[1][1] + theta[1][2]) -
                                  r13 * math.cos(theta[1][1] + theta[1][2])) / math.cos(theta[1][3]),
                                 -r13 * math.sin(theta[1][1] + theta[1][2]) -
                                 r33 * math.cos(theta[1][1] + theta[1][2]))
    else:
        theta[1][4] = math.atan2(r23 / math.sin(theta[1][3]),
                                 -r13 * math.sin(theta[1][1] + theta[1][2]) -
                                 r33 * math.cos(theta[1][1] + theta[1][2]))
    if abs(math.sin(theta[1][4])) < 0.01:
        theta[1][5] = math.atan2((r22 * math.sin(theta[1][3]) +
                                  math.cos(theta[1][3]) * (r32 * math.sin(theta[1][1] + theta[1][2]) -
                                                           r12 * math.cos(theta[1][1] + theta[1][2]))) /
                                 math.cos(theta[1][4]),
                                 math.sin(theta[1][3]) * (r32 * math.sin(theta[1][1] + theta[1][2]) -
                                                          r12 * math.cos(theta[1][1] + theta[1][2])) -
                                 math.cos(theta[1][3]) * r22)
    else:
        theta[1][5] = math.atan2((r12 * math.sin(theta[1][1] + theta[1][2]) +
                                  r32 * math.cos(theta[1][1] + theta[1][2])) / math.sin(theta[1][4]),
                                 math.sin(theta[1][3]) * (r32 * math.sin(theta[1][1] + theta[1][2]) -
                                                          r12 * math.cos(theta[1][1] + theta[1][2])) -
                                 math.cos(theta[1][3]) * r22)

    # -----
    theta.append(theta[1].copy())

    theta[2][2] = -math.atan2(k_tmp, -math.sqrt(L3 ** 2 - k_tmp ** 2))
    theta[2][1] = math.atan2((qx - r13 * L4) * (L2 * math.sin(theta[2][2]) - L3) -
                             (qz - r33 * L4) * (L2 * math.cos(theta[2][2])),
                             (qx - r13 * L4) * (L2 * math.cos(theta[2][2])) +
                             (qz - r33 * L4) * (L2 * math.sin(theta[2][2]) - L3)) - theta[2][2]
    if theta[2][1] > math.pi:
        theta[2][1] = theta[2][1] - 2 * math.pi
    if theta[2][1] < -math.pi:
        theta[2][1] = theta[2][1] + 2 * math.pi

    if abs(r23) < DOUBLE_ABS_MIN and abs(r33 * math.sin(theta[2][1] + theta[2][2]) - r13 * math.cos(theta[2][1] +
                                         theta[2][2])) < DOUBLE_ABS_MIN:
        theta[2][4] = 0
        theta[2][3] = 0
    else:
        theta[2][3] = math.atan2(r23, r33 * math.sin(theta[2][1] + theta[2][2]) -
                                 r13 * math.cos(theta[2][1] + theta[2][2]))
        if abs(math.sin(theta[2][3])) < 0.01:
            theta[2][4] = math.atan2((r33 * math.sin(theta[2][1] + theta[2][2]) -
                                      r13 * math.cos(theta[2][1] + theta[2][2])) / math.cos(theta[2][3]),
                                     -r13 * math.sin(theta[2][1] + theta[2][2]) -
                                     r33 * math.cos((theta[2][1] + theta[2][2])))
        else:
            theta[2][4] = math.atan2(r23 / math.sin(theta[2][3]),
                                     -r13 * math.sin(theta[2][1] + theta[2][2]) -
                                     r33 * math.cos((theta[2][1] + theta[2][2])))
    if abs(math.sin(theta[2][4])) < 0.01:
        theta[2][5] = math.atan2((r22 * math.sin(theta[2][3]) +
                                  math.cos(theta[2][3]) * (r32 * math.sin(theta[2][1] + theta[2][2]) -
                                                           r12 * math.cos(theta[2][1] + theta[2][2]))) /
                                 math.cos(theta[2][4]),
                                 math.sin(theta[2][3]) * (r32 * math.sin(theta[2][1] + theta[2][2]) -
                                                          r12 * math.cos(theta[2][1] + theta[2][2])) -
                                 math.cos(theta[2][3]) * r22)
    else:
        theta[2][5] = math.atan2((r12 * math.sin(theta[2][1] + theta[2][2]) +
                                  r32 * math.cos(theta[2][1] + theta[2][2])) / math.sin(theta[2][4]),
                                 math.sin(theta[2][3]) * (r32 * math.sin(theta[2][1] + theta[2][2]) -
                                                          r12 * math.cos(theta[2][1] + theta[2][2])) -
                                 math.cos(theta[2][3]) * r22)

    # -----
    theta.append(theta[2].copy())
    if theta[3][3] > 0:
        theta[3][3] = theta[3][3] - math.pi
    else:
        theta[3][3] = theta[3][3] + math.pi

    if abs(math.sin(theta[3][3])) < 0.01:
        theta[3][4] = math.atan2((r33 * math.sin(theta[3][1] + theta[3][2]) -
                                  r13 * math.cos(theta[3][1] + theta[3][2])) / math.cos(theta[3][3]),
                                 -r13 * math.sin(theta[3][1] + theta[3][2]) -
                                 r33 * math.cos(theta[3][1] + theta[3][2]))
    else:
        theta[3][4] = math.atan2(r23 / math.sin(theta[3][3]),
                                 -r13 * math.sin(theta[3][1] + theta[3][2]) -
                                 r33 * math.cos(theta[3][1] + theta[3][2]))
    if abs(math.sin(theta[3][4])) < 0.01:
        theta[3][5] = math.atan2((r22 * math.sin(theta[3][3]) +
                                  math.cos(theta[3][3]) * (r32 * math.sin(theta[3][1] + theta[3][2]) -
                                                           r12 * math.cos(theta[3][1] + theta[3][2]))) /
                                 math.cos(theta[3][4]),
                                 math.sin(theta[3][3]) * (r32 * math.sin(theta[3][1] + theta[3][2]) -
                                                          r12 * math.cos(theta[3][1] + theta[3][2])) -
                                 math.cos(theta[3][3]) * r22)
    else:
        theta[3][5] = math.atan2((r12 * math.sin(theta[3][1] + theta[3][2]) +
                                  r32 * math.cos(theta[3][1] + theta[3][2])) / math.sin(theta[3][4]),
                                 math.sin(theta[3][3]) * (r32 * math.sin(theta[3][1] + theta[3][2]) -
                                                          r12 * math.cos(theta[3][1] + theta[3][2])) -
                                 math.cos(theta[3][3]) * r22)

    # -----
    theta.append(theta[3].copy())
    if theta[4][0] > 0:
        theta[4][0] = theta[4][0] - math.pi
    else:
        theta[4][0] = theta[4][0] + math.pi
    r12 = math.cos(theta[4][0]) * ox + math.sin(theta[4][0]) * oy
    r22 = -math.sin(theta[4][0]) * ox + math.cos(theta[4][0]) * oy
    r32 = oz
    r13 = math.cos(theta[4][0]) * ax + math.sin(theta[4][0]) * ay
    r23 = -math.sin(theta[4][0]) * ax + math.cos(theta[4][0]) * ay
    r33 = az
    qx = math.cos(theta[4][0]) * px + math.sin(theta[4][0]) * py
    qy = -math.sin(theta[4][0]) * px + math.cos(theta[4][0]) * py
    qz = pz - L1
    k_tmp = ((qx - r13 * L4) ** 2 + (qz - r33 * L4) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2)

    theta[4][2] = math.atan2(0, L3) - math.atan2(k_tmp, math.sqrt(L3 ** 2 - k_tmp ** 2))
    theta[4][1] = math.atan2((qx - r13 * L4) * (L2 * math.sin(theta[4][2]) - L3) -
                             (qz - r33 * L4) * (L2 * math.cos(theta[4][2])),
                             (qx - r13 * L4) * (L2 * math.cos(theta[4][2])) +
                             (qz - r33 * L4) * (L2 * math.sin(theta[4][2]) - L3)) - theta[4][2]
    if theta[4][1] > math.pi:
        theta[4][1] = theta[4][1] - 2 * math.pi
    if theta[4][1] < -math.pi:
        theta[4][1] = theta[4][1] + 2 * math.pi
    if abs(r23) < DOUBLE_ABS_MIN and abs(r33 * math.sin(theta[4][1] + theta[4][2]) - r13 * math.cos(theta[4][1] +
                                         theta[4][2])) < DOUBLE_ABS_MIN:
        theta[4][4] = 0
        theta[4][3] = 0
    else:
        theta[4][3] = math.atan2(r23, r33 * math.sin(theta[4][1] + theta[4][2]) -
                                 r13 * math.cos(theta[4][1] + theta[4][2]))
        if abs(math.sin(theta[4][3])) < 0.01:
            theta[4][4] = math.atan2((r33 * math.sin(theta[4][1] + theta[4][2]) -
                                      r13 * math.cos(theta[4][1] + theta[4][2])) / math.cos(theta[4][3]),
                                     -r13 * math.sin(theta[4][1] + theta[4][2]) -
                                     r33 * math.cos((theta[4][1] + theta[4][2])))
        else:
            theta[4][4] = math.atan2(r23 / math.sin(theta[4][3]),
                                     -r13 * math.sin(theta[4][1] + theta[4][2]) -
                                     r33 * math.cos((theta[4][1] + theta[4][2])))
    if abs(math.sin(theta[4][4])) < 0.01:
        theta[4][5] = math.atan2((r22 * math.sin(theta[4][3]) +
                                  math.cos(theta[4][3]) * (r32 * math.sin(theta[4][1] + theta[4][2]) -
                                                           r12 * math.cos(theta[4][1] + theta[4][2]))) /
                                 math.cos(theta[4][4]),
                                 math.sin(theta[4][3]) * (r32 * math.sin(theta[4][1] + theta[4][2]) -
                                                          r12 * math.cos(theta[4][1] + theta[4][2])) -
                                 math.cos(theta[4][3]) * r22)
    else:
        theta[4][5] = math.atan2((r12 * math.sin(theta[4][1] + theta[4][2]) +
                                  r32 * math.cos(theta[4][1] + theta[4][2])) / math.sin(theta[4][4]),
                                 math.sin(theta[4][3]) * (r32 * math.sin(theta[4][1] + theta[4][2]) -
                                                          r12 * math.cos(theta[4][1] + theta[4][2])) -
                                 math.cos(theta[4][3]) * r22)
    # -----
    theta.append(theta[4].copy())

    if theta[5][3] > 0:
        theta[5][3] = theta[5][3] - math.pi
    else:
        theta[5][3] = theta[5][3] + math.pi

    if abs(math.sin(theta[5][3])) < 0.01:
        theta[5][4] = math.atan2((r33 * math.sin(theta[5][1] + theta[5][2]) -
                                  r13 * math.cos(theta[5][1] + theta[5][2])) / math.cos(theta[5][3]),
                                 -r13 * math.sin(theta[5][1] + theta[5][2]) -
                                 r33 * math.cos(theta[5][1] + theta[5][2]))
    else:
        theta[5][4] = math.atan2(r23 / math.sin(theta[5][3]),
                                 -r13 * math.sin(theta[5][1] + theta[5][2]) -
                                 r33 * math.cos(theta[5][1] + theta[5][2]))
    if abs(math.sin(theta[5][4])) < 0.01:
        theta[5][5] = math.atan2((r22 * math.sin(theta[5][3]) +
                                  math.cos(theta[5][3]) * (r32 * math.sin(theta[5][1] + theta[5][2]) -
                                                           r12 * math.cos(theta[5][1] + theta[5][2]))) /
                                 math.cos(theta[5][4]),
                                 math.sin(theta[5][3]) * (r32 * math.sin(theta[5][1] + theta[5][2]) -
                                                          r12 * math.cos(theta[5][1] + theta[5][2])) -
                                 math.cos(theta[5][3]) * r22)
    else:
        theta[5][5] = math.atan2((r12 * math.sin(theta[5][1] + theta[5][2]) +
                                  r32 * math.cos(theta[5][1] + theta[5][2])) / math.sin(theta[5][4]),
                                 math.sin(theta[5][3]) * (r32 * math.sin(theta[5][1] + theta[5][2]) -
                                                          r12 * math.cos(theta[5][1] + theta[5][2])) -
                                 math.cos(theta[5][3]) * r22)

    # -----
    theta.append(theta[5])

    theta[6][2] = -math.atan2(k_tmp, -math.sqrt(L3 ** 2 - k_tmp ** 2))
    theta[6][1] = math.atan2((qx - r13 * L4) * (L2 * math.sin(theta[6][2]) - L3) -
                             (qz - r33 * L4) * (L2 * math.cos(theta[6][2])),
                             (qx - r13 * L4) * (L2 * math.cos(theta[6][2])) +
                             (qz - r33 * L4) * (L2 * math.sin(theta[6][2]) - L3)) - theta[6][2]
    if theta[6][1] > math.pi:
        theta[6][1] = theta[6][1] - 2 * math.pi
    if theta[6][1] < -math.pi:
        theta[6][1] = theta[6][1] + 2 * math.pi

    if abs(r23) < DOUBLE_ABS_MIN and abs(r33 * math.sin(theta[6][1] + theta[6][2]) - r13 * math.cos(theta[6][1] +
                                         theta[6][2])) < DOUBLE_ABS_MIN:
        theta[6][4] = 0
        theta[6][3] = 0
    else:
        theta[6][3] = math.atan2(r23, r33 * math.sin(theta[6][1] + theta[6][2]) -
                                 r13 * math.cos(theta[6][1] + theta[6][2]))
        if abs(math.sin(theta[6][3])) < 0.01:
            theta[6][4] = math.atan2((r33 * math.sin(theta[6][1] + theta[6][2]) -
                                      r13 * math.cos(theta[6][1] + theta[6][2])) / math.cos(theta[6][3]),
                                     -r13 * math.sin(theta[6][1] + theta[6][2]) -
                                     r33 * math.cos((theta[6][1] + theta[6][2])))
        else:
            theta[6][4] = math.atan2(r23 / math.sin(theta[6][3]),
                                     -r13 * math.sin(theta[6][1] + theta[6][2]) -
                                     r33 * math.cos((theta[6][1] + theta[6][2])))
    if abs(math.sin(theta[6][4])) < 0.01:
        theta[6][5] = math.atan2((r22 * math.sin(theta[6][3]) +
                                  math.cos(theta[6][3]) * (r32 * math.sin(theta[6][1] + theta[6][2]) -
                                                           r12 * math.cos(theta[6][1] + theta[6][2]))) /
                                 math.cos(theta[6][4]),
                                 math.sin(theta[6][3]) * (r32 * math.sin(theta[6][1] + theta[6][2]) -
                                                          r12 * math.cos(theta[6][1] + theta[6][2])) -
                                 math.cos(theta[6][3]) * r22)
    else:
        theta[6][5] = math.atan2((r12 * math.sin(theta[6][1] + theta[6][2]) +
                                  r32 * math.cos(theta[6][1] + theta[6][2])) / math.sin(theta[6][4]),
                                 math.sin(theta[6][3]) * (r32 * math.sin(theta[6][1] + theta[6][2]) -
                                                          r12 * math.cos(theta[6][1] + theta[6][2])) -
                                 math.cos(theta[6][3]) * r22)

    # -----
    theta.append(theta[6].copy())
    if theta[7][3] > 0:
        theta[7][3] = theta[7][3] - math.pi
    else:
        theta[7][3] = theta[7][3] + math.pi

    if abs(math.sin(theta[7][3])) < 0.01:
        theta[7][4] = math.atan2((r33 * math.sin(theta[7][1] + theta[7][2]) -
                                  r13 * math.cos(theta[7][1] + theta[7][2])) / math.cos(theta[7][3]),
                                 -r13 * math.sin(theta[7][1] + theta[7][2]) -
                                 r33 * math.cos(theta[7][1] + theta[7][2]))
    else:
        theta[7][4] = math.atan2(r23 / math.sin(theta[7][3]),
                                 -r13 * math.sin(theta[7][1] + theta[7][2]) -
                                 r33 * math.cos(theta[7][1] + theta[7][2]))
    if abs(math.sin(theta[7][4])) < 0.01:
        theta[7][5] = math.atan2((r22 * math.sin(theta[7][3]) +
                                  math.cos(theta[7][3]) * (r32 * math.sin(theta[7][1] + theta[7][2]) -
                                                           r12 * math.cos(theta[7][1] + theta[7][2]))) /
                                 math.cos(theta[7][4]),
                                 math.sin(theta[7][3]) * (r32 * math.sin(theta[7][1] + theta[7][2]) -
                                                          r12 * math.cos(theta[7][1] + theta[7][2])) -
                                 math.cos(theta[7][3]) * r22)
    else:
        theta[7][5] = math.atan2((r12 * math.sin(theta[7][1] + theta[7][2]) +
                                  r32 * math.cos(theta[7][1] + theta[7][2])) / math.sin(theta[7][4]),
                                 math.sin(theta[7][3]) * (r32 * math.sin(theta[7][1] + theta[7][2]) -
                                                          r12 * math.cos(theta[7][1] + theta[7][2])) -
                                 math.cos(theta[7][3]) * r22)

    d_theta = np.matrix([[0], [math.pi / 2], [math.pi / 2], [math.pi], [0], [0]], dtype=float)
    for i in range(len(theta)):
        theta[i] = theta[i] + d_theta
        theta[i][4] = -theta[i][4]
        for j in range(6):
            if theta[i][j] > math.pi:
                theta[i][j] = theta[i][j] - 2 * math.pi
            if theta[i][j] < -math.pi:
                theta[i][j] = theta[i][j] + 2 * math.pi
    # print(theta)
    res = []
    for i in range(len(theta)):
        flag = True
        for j in range(6):
            if theta[i][j] >= qMax_r[j] or theta[i][j] <= qMin_r[j]:
                flag = False
                break
        if flag:
            res.append(theta[i])
            # return theta[i]
    # print(res)
    if len(res) > 0:
        m = []
        for i in range(len(res)):
            tmp = 0
            for j in range(len(initial_theta)):
                tmp += (res[i][j] - initial_theta[j]) ** 2
            m.append(tmp)
        min_ = m[0]
        index = 0
        for i in range(len(m)):
            if m[i] < min_:
                min_ = m[i]
                index = i
        return res[index], 1
    return initial_theta, 0
