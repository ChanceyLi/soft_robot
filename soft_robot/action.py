import model
import control
import my_serial
import model_function

"""
机器人控制序列，动作模式、流程等实现。
"""


class Action:
    def __init__(self, lst=None):
        self.controller = control.Control(lst)

    def linear_interpolation_solving_angles(self, endPoint, times=10):
        """线性插值求解角度序列，采用预测控制方法
        Args:
            endPoint 直线插值的终点
            param times 插值的点的个数
        Returns:
            返回差值的角度序列
        """
        return self.controller.m.inverse_kinematics_opt(endPoint, times)

    def linear_action_with_thetas(self, thetas):
        """线性运动（角度序列）
        Args:
            thetas 角度序列
        Returns:
            None
        """
        self.controller.auto_run(0, 1, 2, thetas)

    def linear_action_with_endpoint(self, endPoint):
        """线性运动（终点位置）
        Args:
            endPoint 终点
        Returns:
            None
        """
        thetas = self.linear_interpolation_solving_angles(endPoint)
        self.linear_action_with_thetas(thetas)

    def set_radians_with_push_mode(self, position, theta_soft):
        """在按压模式下已知最终位置和软体弯曲度求解机器人姿态（角度）
        Args:
            position 终点（六维：位置加姿态）
            theta_soft 软体部分的弯曲度(弧度）
        Returns:
            thetas 机器人姿态（七维弧度）
        """
        delta = theta_soft * self.controller.m.D / self.controller.m.L
        r = model_function.delta_vector(delta, self.controller.m.L, self.controller.m.D)
        p = position.copy()
        p[1] -= r[1]
        p[2] += r[2]
        p[3] += theta_soft
        T = model.position2trans_matrix(p)
        theta = [0, 0, 0, 0, 0, 0, theta_soft]
        tmp, _ = model.inverse_kinematic(T, self.controller.m.get_radian()[:6])
        for i in range(6):
            theta[i] = tmp[i]
        self.controller.m.set_radians(theta)
        return theta

    def move_and_push(self, position, theta_soft):
        """移动并按压
        Args:
            position 移动的末端位置
            theta_soft 软体末端的弯曲度
        Returns:
            Null
        """
        theta = self.set_radians_with_push_mode(position, theta_soft)
        self.controller.initial()
        self.controller.send(0, 1, theta)
        dz = 2
        thetas = []
        p = position.copy()
        for i in range(5):
            p[2] -= dz
            theta = self.set_radians_with_push_mode(p, theta_soft)
            thetas.append(theta)
        self.controller.auto_run(0, 1, 2, thetas)
