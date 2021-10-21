"""
    建模文件
"""

import init
import model_function
import numpy as np


class Model(init.Parameters):
    theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    __d = 10  # soft body distance
    __l = 50  # soft body length
    delta = 0.1  # soft body length change rate
    M_soft = np.matrix([4, 4], dtype=float)
    T = np.matrix([4, 4], dtype=float)
    T_soft = np.matrix([4, 4], dtype=float)
    T_all = []

    def __init__(self, theta, delta=0.1):
        super().__init__()
        self.theta[:6] = theta
        self.delta = delta
        self.theta[6] = self.delta * self.__l / self.__d
        self.change_q7(model_function.delta_vector(self.delta, self.__l, self.__d))
        self.M_soft = self.M.copy()
        self.M_soft[:3, 3] = self.q[6]
        self.init_S()

        self.J = np.zeros((6, len(self.omega)), dtype=float)

        self.forward_matrix()
        self.jacobi()

    def forward_matrix(self):
        """
        :param
        :前向齐次转移矩阵
        """
        T = []
        for i in range(len(self.omega)):
            T.append(model_function.T(model_function.screw(self.omega[i]), self.q[i], self.theta[i]))

        Te = T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ T[5] @ self.M
        Te_soft = T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ T[5] @ T[6] @ self.M_soft
        self.T_all = T
        self.T = Te
        self.T_soft = Te_soft

    def jacobi(self):
        """
        :param
        :雅可比矩阵
        """
        J = np.zeros((6, len(self.omega)), dtype=float)
        for i in range(len(self.omega)):
            tmp = np.eye(4)
            for j in range(i):
                tmp = tmp @ self.T_all[j]
            ttmp = np.zeros((6, 6), dtype=float)
            ttmp[:3, :3] = tmp[:3, :3]
            ttmp[3:, 3:] = tmp[:3, :3]
            ttmp[3:, :3] = model_function.screw([tmp[0, 3], tmp[1, 3], tmp[2, 3]]) @ tmp[:3, :3]
            J[:, i:i+1] = ttmp @ self.S[i]
        self.J = J


def model_gurobi(queue):
    x0 = queue[0]
    N = len(queue) - 1
    XS = np.zeros((6 * N, 1), dtype=float)
    T = 0.1  # v = (x(i+1) - x(i)) / T
    my_robot = Motivation()
    my_robot.set_position(x0)
    q = my_robot.get_radian()
    J = my_robot.get_jacobi()
    B = np.zeros((6 * N, 6 * N), dtype=float)
    I_N = np.zeros((6 * N, 6 * N), dtype=float)
    I = np.eye(6 * N, 6 * N)
    for i in range(N):
        for j in range(N):
            if i >= j:
                B[6 * i:6 * i + 6, 6 * j:6 * j + 6] = T * J
                I_N[6 * i:6 * i + 6, 6 * j:6 * j + 6] = np.eye(6, 6)
    I_NN = I_N @ I_N

    X0 = np.zeros((6 * N, 1), dtype=float)
    Q = np.zeros((6 * N, 1), dtype=float)
    Lq = np.zeros((6 * N, 1), dtype=float)  # 角度约束
    Hq = np.zeros((6 * N, 1), dtype=float)
    Lw = np.zeros((6 * N, 1), dtype=float)  # 角速度约束
    Hw = np.zeros((6 * N, 1), dtype=float)
    for i in range(N):
        for j in range(6):
            X0[6 * i + j, 0] = x0[j]
            XS[6 * i + j, 0] = queue[i + 1][j]
            Q[6 * i + j, 0] = q[j]
            Lq[6 * i + j, 0] = lower[j] * np.pi / 180
            Hq[6 * i + j, 0] = upper[j] * np.pi / 180
            Lw[6 * i + j, 0] = vMin[j]
            Hw[6 * i + j, 0] = vMax[j]
    Gq = np.zeros((12 * N, 6 * N), dtype=float)
    Gq[:6 * N, ] = I_NN
    Gq[6 * N:, ] = -I_NN
    Bq = np.zeros((12 * N, 1), dtype=float)
    Bq[:6 * N, ] = Hq - Q
    Bq[6 * N:, ] = Q - Lq
    Gw = np.zeros((12 * N, 6 * N), dtype=float)
    Gw[:6 * N, ] = T * I_N
    Gw[6 * N:, ] = -T * I_N
    Bw = np.zeros((12 * N, 1), dtype=float)
    Bw[:6 * N, ] = Hw
    Bw[6 * N:, ] = -Lw

    g_p = np.zeros((12, 6), dtype=float)
    g_p[0:6, 0:6] = np.eye(6)
    g_p[6:12, 0:6] = -np.eye(6)
    b_p = np.matrix([[x_max], [y_max], [z_max], [0], [0], [0],
                     [-x_min], [-y_min], [-z_min], [0], [0], [0]])
    Gp = np.zeros((12 * N, 6 * N), dtype=float)  # 位置约束
    Bp = np.zeros((12 * N, 1), dtype=float)
    for i in range(N):
        Gp[12 * i:12 * i + 12, 6 * i:6 * i + 6] = g_p
        Bp[12 * i:12 * i + 12, ] = b_p
    Bp = Bp - Gp @ X0
    Gp = Gp @ B @ I_N

    alpha = 1
    G = np.zeros((36 * N, 6 * N), dtype=float)
    G[:12 * N, ] = Gp
    G[12 * N:24 * N, ] = Gq
    G[24 * N:, ] = Gw
    B_ = np.zeros((36 * N, 1), dtype=float)
    B_[:12 * N, ] = Bp
    B_[12 * N:24 * N, ] = Bq
    B_[24 * N:, ] = Bw
    # G_a, B_a = G.A, B_.A
    Hess = (B @ I_N).T @ (B @ I_N) + alpha * I
    c = (X0 - XS).T @ B @ I_N
    u = cvxopt_solve_qp(Hess, c.T, G, B_, None, None)
    print(u)
