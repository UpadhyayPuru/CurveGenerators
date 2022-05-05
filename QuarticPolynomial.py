import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
class QuarticPolynomial:
    def __init__(self,init_pos,init_vel,init_acc,final_vel,final_acc,T):

        # s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
        self.a0 = init_pos
        self.a1 = init_vel
        self.a2 = init_acc/2

        A = np.array([[3*(T**2), 4*(T**3)],[6*T, 12*(T**2)]])
        B = np.array([[final_vel-init_vel-init_acc*T],[final_acc - init_acc]])

        x = np.linalg.solve(A,B)

        self.a3 = x[0]
        self.a4 = x[1]

    def cal_position(self,t):

        xt = self.a0 + self.a1*t + self.a2*(t**2) + self.a3*(t**3) + self.a4*(t**4)

        return xt

    def cal_velocity(self,t):

        vt = self.a1 + 2*self.a2*t + 3*self.a3*(t**2) + 4*self.a4*(t**3)

        return vt

    def cal_acceleration(self,t):

        at = 2*self.a2 + 6*self.a3*t + 12*self.a4*(t**2)

        return at

    def cal_jerk(self,t):

        jt = 6*self.a3 + 24*self.a4*t

        return jt


def main():
    x0 = 0
    v0 = 0
    a0 = 0
    v1 = 4
    a1 = 1
    T = 5
    traj = QuarticPolynomial(x0,v0,a0,v1,a1,T)
    pos = []
    time = []
    dt = 0
    while dt<5:
        pos.append(traj.cal_position(dt))
        time.append(dt)
        dt = dt + 0.1

    plt.plot(time,pos)
    plt.show()



if __name__=="__main__":
    main()

