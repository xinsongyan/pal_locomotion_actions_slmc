import numpy as np
from casadi import *
from scipy import interpolate
import time
from scipy.interpolate import CubicSpline, UnivariateSpline, interp1d
import math
# import rospy

# whole body controller runs at 500 HZ
g = 9.81
z_c = 0.87
step_duration = [2.0, 2.0]  # index 0 is double support, index 1 is single support
# zmp reference: left is positive, right is negative. Based on world coordinates
cop_y_min, cop_y_max = -0.085, 0.085
com_x_vel = 0.025

support_indexes = np.array([0, -1, 0, 1, 0, -1, 0, 1, 0])
support_duartions = np.array([step_duration[int(abs(support_indexes[i]))] for i in range(support_indexes.size)])
support_duartions_acm = np.concatenate(([0], np.cumsum(support_duartions)))
# get foot placement at each step: left_x, left_y, left_z, right_x, right_y, right_z
foot_placement = np.zeros((6, support_indexes.size))
# ---- Optimization problem ---------
opti = Opti()  # Optimization problem
T = np.sum(support_duartions)

number_of_knots = 100
num_of_segs = number_of_knots - 1
dt = T / num_of_segs

cop_ref_y = np.zeros((1, number_of_knots))
cop_ref_x = np.zeros((1, number_of_knots))
single_support_count = 0
# initial foot placement
foot_placement[1,0] = cop_y_max
foot_placement[4,0] = cop_y_min
for i in range(support_indexes.size):

    if support_indexes[i] == -1: # right support
        p_step_y = cop_y_min
        single_support_count += 1
        foot_placement[3:, i] = foot_placement[3:, i-1]
        foot_placement[0, i] = com_x_vel * single_support_count * (np.sum(step_duration)/step_duration[0])
        foot_placement[1, i] = cop_y_max
        cop_ref_x[:, int(support_duartions_acm[i] / dt):int(support_duartions_acm[i + 1] / dt)] = foot_placement[3, i]
        cop_ref_y[:, int(support_duartions_acm[i] / dt):int(support_duartions_acm[i + 1] / dt)] = p_step_y
        if i == support_indexes.size - 2:
            foot_placement[0, i] = foot_placement[3, i]

    elif support_indexes[i] == 1: # left support
        p_step_y = cop_y_max
        single_support_count += 1
        foot_placement[:3, i] = foot_placement[:3, i-1]
        foot_placement[3, i] = com_x_vel * single_support_count * (np.sum(step_duration)/step_duration[0])
        foot_placement[4, i] = cop_y_min
        cop_ref_x[:, int(support_duartions_acm[i] / dt):int(support_duartions_acm[i + 1] / dt)] = foot_placement[0, i]
        cop_ref_y[:, int(support_duartions_acm[i] / dt):int(support_duartions_acm[i + 1] / dt)] = p_step_y
        if i == support_indexes.size - 2:
            foot_placement[3, i] = foot_placement[0, i]

    elif support_indexes[i] == 0:  # double support
        p_step_y = 0.0
        if i != 0:
            foot_placement[:, i] = foot_placement[:, i - 1]
        # at last step, just make both sides at same x pos

    if support_indexes[i] == 0:
        if i != 0 and i != support_indexes.size - 1:
            duration_num_current_step = math.ceil(support_duartions[i] / dt)
            # to be changed
            if support_indexes[i - 1] == -1:
                for j in range(int(duration_num_current_step+1)):
                    cop_ref_x[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[3, i-1] + (
                                foot_placement[0, i] - foot_placement[3, i-1]) * j / duration_num_current_step
                    cop_ref_y[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[4, i-1] + (
                                foot_placement[1, i] - foot_placement[4, i-1]) * j / duration_num_current_step
            else:
                for j in range(int(duration_num_current_step+1)):
                    cop_ref_x[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[0, i-1] + (
                                foot_placement[3, i] - foot_placement[0, i-1]) * j / duration_num_current_step
                    cop_ref_y[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[1, i-1] + (
                                foot_placement[4, i] - foot_placement[1, i-1]) * j / duration_num_current_step
        if i == support_indexes.size-1:
            if support_indexes[i - 1] == -1:
                for j in range(int(duration_num_current_step+1)):
                    cop_ref_x[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[3, i-1] + (
                                foot_placement[0, i] - foot_placement[3, i-1]) * j / duration_num_current_step
                    # cop_ref_y[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[4, i-1] + (
                    #             foot_placement[1, i-1] - foot_placement[4, i-1]) * j / duration_num_current_step
            else:
                for j in range(int(duration_num_current_step+1)):
                    cop_ref_x[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[0, i-1] + (
                                foot_placement[3, i] - foot_placement[0, i-1]) * j / duration_num_current_step
                    # cop_ref_y[:, int(support_duartions_acm[i] / dt) + j] = foot_placement[1, i-1] + (
                    #             foot_placement[4, i-1] - foot_placement[1, i-1]) * j / duration_num_current_step

print('foot place is ', foot_placement.T)
# ---- decision variables ---------
# ---- states ---------
X = opti.variable(3, number_of_knots)
x = X[0, :]
xd = X[1, :]
xdd = X[2, :]

Y = opti.variable(3, number_of_knots)
y = Y[0, :]
yd = Y[1, :]
ydd = Y[2, :]

# ---- inputs ---------
U_X = opti.variable(1, number_of_knots)
U_Y = opti.variable(1, number_of_knots)


# ---- dynamics ---------
def f_x(X, U_X):
    x_, xd_, xdd_ = X[0], X[1], X[2]
    return vertcat(xd_, xdd_, U_X)

def f_y(Y, U_Y):
    y_, yd_, ydd_ = Y[0], Y[1], Y[2]
    return vertcat(yd_, ydd_, U_Y)


# ---- objective ---------
Q, R = 1.0, 1e-6
opti.minimize(Q * (sumsqr(y - z_c / g * ydd - cop_ref_y)+sumsqr(x - z_c / g * xdd - cop_ref_x))
              + R * (sumsqr(U_X)+sumsqr(U_Y)))

# ---- collocation constraints ---------
for k in range(number_of_knots-1):
    opti.subject_to(X[:, k + 1] - X[:, k] == 0.5 * dt * (f_x(X[:, k + 1], U_X[:, k + 1]) + f_x(X[:, k], U_X[:, k])))
    opti.subject_to(Y[:, k + 1] - Y[:, k] == 0.5 * dt * (f_y(Y[:, k + 1], U_Y[:, k + 1]) + f_y(Y[:, k], U_Y[:, k])))

# ---- boundary conditions --------
x0, xd0, xdd0 = 0, 0, 0
opti.subject_to(x[0] == x0)
opti.subject_to(xd[0] == xd0)
opti.subject_to(xdd[0] == xdd0)

y0, yd0, ydd0 = 0, 0, 0
opti.subject_to(y[0] == y0)
opti.subject_to(yd[0] == yd0)
opti.subject_to(ydd[0] == ydd0)

y_end, yd_end, ydd_end = 0, 0, 0
opti.subject_to(y[-1] == y_end)
opti.subject_to(yd[-1] == yd_end)
opti.subject_to(ydd[-1] == ydd_end)

# ---- initial guess for solver ---
opti.set_initial(X, 0)
opti.set_initial(Y, 0)
opti.set_initial(U_X, 0)
opti.set_initial(U_Y, 0)

time_start = time.time()
options = {"ipopt.print_level": 0, "print_out": False, "print_in": False, "print_time": False}  # "verbose": True,

# ---- solve NLP              ------
opti.solver("ipopt", options)  # set numerical backend, options
sol = opti.solve()  # actual solve
time_end = time.time()
print('Time useage is ', time_end - time_start)



time = [0]
time = np.concatenate((time, np.ones(num_of_segs)*dt))
time = np.cumsum(time)
f_pos_x = interp1d(time, sol.value(x), fill_value=(sol.value(x)[0], sol.value(x)[-1]), bounds_error=False)
f_vel_x = interp1d(time, sol.value(xd), fill_value=(sol.value(xd)[0], sol.value(xd)[-1]), bounds_error=False)
f_acc_x = interp1d(time, sol.value(xdd), fill_value=(sol.value(xdd)[0], sol.value(xdd)[-1]), bounds_error=False)
f_pos_y = interp1d(time, sol.value(y), fill_value=(sol.value(y)[0], sol.value(y)[-1]), bounds_error=False)
f_vel_y = interp1d(time, sol.value(yd), fill_value=(sol.value(yd)[0], sol.value(yd)[-1]), bounds_error=False)
f_acc_y = interp1d(time, sol.value(ydd), fill_value=(sol.value(ydd)[0], sol.value(ydd)[-1]), bounds_error=False)
f_zmp_x = interp1d(time, cop_ref_x[0, :], fill_value=(cop_ref_x[0, 0], cop_ref_x[0, -1]), bounds_error=False)
f_zmp_y = interp1d(time, cop_ref_y[0, :], fill_value=(cop_ref_y[0, 0], cop_ref_y[0, -1]), bounds_error=False)

freq = 500
sample_num = int(freq * T)
timetime = np.linspace(0, T, sample_num)

x_pos = f_pos_x(timetime)
x_vel = f_vel_x(timetime)
x_acc = f_vel_x(timetime)

y_pos = f_pos_y(timetime)
y_vel = f_vel_y(timetime)
y_acc = f_vel_y(timetime)

zmp_x = f_zmp_x(timetime)
zmp_y = f_zmp_y(timetime)

# ---- Do sampling and save results


# pos, vel, acc
state_sample_x = np.zeros((3, sample_num))
state_sample_y = np.zeros((3, sample_num))
state_sample_z = np.zeros((3, sample_num))
zmp_sample = np.zeros((3, sample_num))

state_sample_x[0, :] = x_pos
state_sample_x[1, :] = x_vel
state_sample_x[2, :] = x_acc
state_sample_y[0, :] = y_pos
state_sample_y[1, :] = y_vel
state_sample_y[2, :] = y_acc
state_sample_z[0, :] = z_c*np.ones(sample_num)
zmp_sample[0, :] = zmp_x
zmp_sample[1, :] = zmp_y
a = np.asarray((timetime, state_sample_x[0, :], state_sample_y[0, :], state_sample_z[0, :], state_sample_x[1, :],
                state_sample_y[1, :], state_sample_z[1, :], state_sample_x[2, :], state_sample_y[2, :], state_sample_z[2, :])).T

np.savetxt('com_trajectory.csv', a, delimiter=',')

np.savetxt('support_durations.csv', support_duartions, delimiter=',')

np.savetxt('support_indexes.csv', support_indexes, delimiter=',')

np.savetxt('foot_placement.csv', foot_placement.T, delimiter=',')
b = np.asarray((timetime, zmp_sample[0, :], zmp_sample[1, :], zmp_sample[2, :])).T
np.savetxt('zmp_trajectory.csv', b, delimiter=',')


################ Plotting the results ###################################
# import matplotlib.pyplot as plt
# #
# plt.figure(1)
# plt.plot(time, cop_ref_x.T, 'r.', label='cop_ref_x')
# plt.plot(time, sol.value(x), 'g-', label='com')
# # plt.plot(time, sol.value(xd), 'g-', label='dcom')
# # plt.plot(timetime, state_sample_x[0, :], 'r.', label='sample_com')
# # plt.plot(timetime, zmp_sample[0, :], 'r.', label='sample_zmp')
# plt.legend(loc='best')
# plt.xlabel('time (s)')
# plt.ylabel('pos (m)')
# plt.title("ZMP Tracking Control")
# plt.grid()
# plt.show()
# plt.figure(1)
# plt.plot(time, cop_ref_y.T, 'r.')
# plt.plot(time, sol.value(y), 'g-', label='com_y')
# # plt.plot(time, sol.value(yd), 'g-', label='com_yd')
# # plt.plot(time, sol.value(ydd), 'g-', label='com_ydd')
# plt.plot(timetime, state_sample_y[0, :], 'r.', label='sample_com')
# plt.plot(timetime, zmp_sample[1, :], 'r.', label='sample_zmp')
# plt.legend(loc='best')
# plt.xlabel('time (s)')
# plt.ylabel('pos (m)')
# plt.title("ZMP Tracking Control")
# plt.grid()
# plt.show()

# plt.figure(1)
# time = np.linspace(0, T, num_of_segs)

###############################################################
import trajectory_publisher

trajectory_publisher.publish_all()
