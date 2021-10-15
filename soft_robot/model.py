import init
import model_function
import numpy as np

# model about soft robot


class Model(init.Parameters):
    theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    __d = 10  # soft body distance
    __l = 50  # soft body length
    delta = 0.1  # soft body length change rate
    M_soft = np.matrix([4, 4], dtype=float)
    T = np.matrix([4, 4], dtype=float)
    T_soft = np.matrix([4, 4], dtype=float)

    def __init__(self, theta, delta=0.1):
        super().__init__()
        self.theta[:6] = theta
        self.delta = delta
        self.theta[6] = self.delta * self.__l / self.__d
        self.change_q7(model_function.delta_vector(self.delta, self.__l, self.__d))
        self.M_soft = self.M.copy()
        self.M_soft[:3, 3] = self.q7

    def forward_matrix(self):
        T01 = model_function.T(model_function.screw(self.omega1), self.q1, self.theta[0])
        T12 = model_function.T(model_function.screw(self.omega2), self.q2, self.theta[1])
        T23 = model_function.T(model_function.screw(self.omega3), self.q3, self.theta[2])
        T34 = model_function.T(model_function.screw(self.omega4), self.q4, self.theta[3])
        T45 = model_function.T(model_function.screw(self.omega5), self.q5, self.theta[4])
        T56 = model_function.T(model_function.screw(self.omega6), self.q6, self.theta[5])
        T67 = model_function.T(model_function.screw(self.omega7), self.q7, self.theta[6])

        Te = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ self.M
        Te_soft = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67 @ self.M_soft
        self.T = Te
        self.T_soft = Te_soft

