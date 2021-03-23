import numpy as np
from scipy.interpolate import CubicSpline, UnivariateSpline, interp1d
# from pybullet_debug_plot import *
from script.utils.pybullet_debug_plot import *

class CoMTrajectory:
    def __init__(self, time, pos, vel=None, acc=None, debug_plot=True):
        self.time = time
        self.pos = pos
        self.vel = vel
        self.acc = acc

        self.f_pos = interp1d(time, pos, fill_value=(pos[:,0], pos[:,-1]), bounds_error=False)
        self.f_vel = interp1d(time, vel, fill_value=(vel[:,0], vel[:,-1]), bounds_error=False)
        self.f_acc = interp1d(time, acc, fill_value=(acc[:,0], acc[:,-1]), bounds_error=False)

        # self.f_pos = CubicSpline(time, pos.T)
        # self.f_vel = CubicSpline(time, vel.T)
        # self.f_acc = CubicSpline(time, acc.T)


        if debug_plot is True:
            addDebugTrajectory3D(self.pos, color=[1, 0, 0])
            # .addDebugTrajectory2D(self.pos, color=[1, 0, 0])

    def __call__(self, t):
        return {"pos": self.f_pos(t),
                 "vel": self.f_vel(t),
                 "acc": self.f_acc(t)}

    def get_trajectory_array(self, dt=0.001):
        time = np.linspace(self.time[0], self.time[-1], int((self.time[-1]-self.time[0])/dt))
        return np.vstack((time, self.f_pos(time), self.f_vel(time), self.f_acc(time))).T


# class SwingTrajectory:
#     def __init__(self, time, pos, vel=None, acc=None, debug_plot=True):
#         self.time = time
#         self.pos = pos
#         self.vel = vel
#         self.acc = acc
#
#
#         self.f_pos = interp1d(time, pos, fill_value=(pos[:,0], pos[:,-1]), bounds_error=False)
#         if vel is not None:
#             self.f_vel = interp1d(time, vel, fill_value=(vel[:,0], vel[:,-1]), bounds_error=False)
#         if acc is not None:
#             self.f_acc = interp1d(time, acc, fill_value=(acc[:,0], acc[:,-1]), bounds_error=False)
#
#         if debug_plot is True:
#             addDebugTrajectory3D(self.pos, color=[1, 0, 0])
#
#     def __call__(self, t):
#         return {"pos": self.f_pos(t),
#                  "vel": self.f_vel(t),
#                  "acc": self.f_acc(t)}




class SwingTrajectory:
    def __init__(self, ini_pos, end_pos, mid_height=0.2, duration=0.3, ini_vel=np.zeros(3), end_vel=np.zeros(3), debug_plot=False, debug_plot_dt=0.001, debug_plot_color=[0, 0, 1]):
        self.ini_pos = np.array(ini_pos)
        self.end_pos = np.array(end_pos)
        self.mid_pos = (self.ini_pos+self.end_pos)/2
        self.mid_height = self.mid_pos[2] + mid_height
        self.duration = duration

        self.mid_pos = 0.5 * (self.ini_pos + self.end_pos)
        self.mid_pos[2] = mid_height

        self.ini_vel = ini_vel
        self.end_vel = end_vel

        self.cubic_spline = CubicSpline(x=[0.0, self.duration / 2.0, self.duration],
                                            y=np.array([self.ini_pos, self.mid_pos, self.end_pos]),
                                            axis=0,
                                            bc_type=((1, self.ini_vel), (1, self.end_vel)))

        if debug_plot is True:
           addDebugTrajectory3D(self.pos(np.arange(0, self.duration, debug_plot_dt)), color=debug_plot_color)


    def __call__(self, t):
        return {"pos": self.pos(t),
                "vel": self.vel(t),
                "acc": self.acc(t)}

    def x(self, t):
        return self.cubic_spline(t)[:,0]

    def y(self, t):
        return self.cubic_spline(t)[:,1]

    def z(self, t):
        return self.cubic_spline(t)[:,2]

    def pos(self, t):
        return self.cubic_spline(t)

    def vel(self, t):
        return self.cubic_spline(t, 1)

    def acc(self, t):
        return self.cubic_spline(t, 2)

    def interpolate(self, t):
        return {"pos": self.cubic_spline(t),
                "vel": self.cubic_spline(t, 1),
                "acc": self.cubic_spline(t, 2)}



class CubicPolynomial:
    def __init__(self, x0, xd0, x1, xd1, T):
        self.T = T
        self.a0, self.a1, self.a2, self.a3 = self.coefficients_from_boundray_condition(x0, xd0, x1, xd1, T)

    def __call__(self, t):
        return self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3

    def coefficients_from_boundray_condition(self,x0, xd0, x1, xd1, T):
        a0 = x0
        a1 = xd0
        a2 = -1/T**2*(3*(x0 - x1) + T*(2*xd0 + xd1))
        a3 = 1/T**3*(2*(x0 - x1) + T*(xd0 + xd1))
        return a0, a1, a2, a3



    def plot(self, N=100):
        import matplotlib.pyplot as plt
        tt = np.linspace(0,self.T,N)
        xx = self.__call__(tt)
        plt.plot(tt,xx)
        plt.show()


def solve_symbolic_cubic_polynomial():
    from sympy import var
    from sympy import diff, solve, sympify
    a0, a1, a2, a3 = var('a0 a1 a2 a3')
    x0, xd0, x1, xd1, T = var('x0 xd0 x1 xd1 T')
    x = var('x')

    f = a0 + a1*x + a2*x**2 + a3*x**3
    fd = diff(f,x)

    E1 = f.subs(x,0) - x0
    E2 = fd.subs(x,0) - xd0
    E3 = f.subs(x,T) - x1
    E4 = fd.subs(x,T) - xd1
    sols = solve([E1, E2, E3, E4], [a0, a1, a2, a3])
    print(sympify(sols))


def solve_symbolic_quintic_polynomial():
    from sympy import var
    from sympy import diff, solve, sympify
    a0, a1, a2, a3, a4, a5 = var('a0 a1 a2 a3 a4 a5')
    x0, xd0, xm, xdm, x1, xd1, T = var('x0 xd0 xm xdm x1 xd1  T')
    x = var('x')

    f = a0 + a1*x + a2*x**2 + a3*x**3 + a4*x**4 + a5*x**5
    fd = diff(f,x)

    E1 = f.subs(x,0) - x0
    E2 = fd.subs(x,0) - xd0
    E3 = f.subs(x,T/2) - xm
    E4 = fd.subs(x,T/2) - xdm
    E5 = f.subs(x,T) - x1
    E6 = fd.subs(x,T) - xd1
    sols = solve([E1, E2, E3, E4, E5, E6], [a0, a1, a2, a3, a4, a5])
    print(sympify(sols))


if __name__ == "__main__":
    print('main')
    # solve_symbolic_cubic_polynomial()
    # solve_symbolic_quintic_polynomial()
    # cp = CubicPolynomial(x0=0, xd0=0, x1=1, xd1=-1, T=2)
    # cp.plot()

    x = np.array([0,1,2,3])
    y = np.array([1,2,1,2])

    cs = CubicSpline(x,y)
    xx = np.linspace(0,3,100)
    yy = cs(xx)

    interp1d = interp1d(x,y,kind='cubic')
    xxx = xx
    yyy = interp1d(xxx)

    import matplotlib.pyplot as plt
    plt.plot(x,y,'o')
    plt.plot(xx,yy,'r')
    plt.plot(xxx, yyy, 'g--')
    plt.legend(['origin', 'cubic_spline', 'interp1d'])
    plt.show()

