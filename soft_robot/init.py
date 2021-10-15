import numpy as np


class Parameters:
    omega1 = [0, 0, 1]
    omega2 = [1, 0, 0]
    omega3 = [1, 0, 0]
    omega4 = [0, 0, 1]
    omega5 = [1, 0, 0]
    omega6 = [0, 0, 1]
    omega7 = [0, 1, 0]

    q1 = np.matrix([[0], [0], [242]], dtype=float)
    q2 = np.matrix([[0], [0], [242]], dtype=float)
    q3 = np.matrix([[0], [0], [467]], dtype=float)
    q4 = np.matrix([[0], [0], [695.86]], dtype=float)
    q5 = np.matrix([[0], [0], [695.86]], dtype=float)
    q6 = np.matrix([[0], [0], [745.86]], dtype=float)
    q7 = np.matrix([[0], [0], [745.86]], dtype=float)

    M = np.matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 745.86],
                   [0, 0, 0, 1]], dtype=float)

    def __init__(self):
        pass

    def change_q7(self, dv):
        self.q7 = self.q7 + dv
