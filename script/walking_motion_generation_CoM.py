import time
import casadi
from casadi import *
from utils.trajectory import SwingTrajectory, CoMTrajectory
from utils.pybullet_debug_plot import *
import matplotlib.pyplot as plt
np.set_printoptions(precision=3, suppress=True)



class ComMotionPlanner:
    def __init__(self, num_of_phases, robot_param, segments_per_phase=10):
        self.m = robot_param['m']
        self.g = robot_param['g']

        self.foot_length = robot_param['foot_length']
        self.foot_width = robot_param['foot_width']
        self.v1 = vertcat(+self.foot_length / 2, +self.foot_width / 2, 0)
        self.v2 = vertcat(-self.foot_length / 2, +self.foot_width / 2, 0)
        self.v3 = vertcat(-self.foot_length / 2, -self.foot_width / 2, 0)
        self.v4 = vertcat(+self.foot_length / 2, -self.foot_width / 2, 0)
        self.Vs = horzcat(self.v1, self.v2, self.v3, self.v4)

        self.norminal_com_height = robot_param['norminal_com_height']

        self.single_support_duration = robot_param['single_support_duration']
        self.double_support_duration = robot_param['double_support_duration']

        self.solution_saved = False
        self.set_opti(num_of_phases=num_of_phases, segments_per_phase=segments_per_phase)

    def f(self, X, U):
        x, y, z, xd, yd, zd = X[0], X[1], X[2], X[3], X[4], X[5]
        px, py, pz, fz = U[0], U[1], U[2], U[3]
        xdd = (x - px) * fz / (z-pz) / self.m
        ydd = (y - py) * fz / (z-pz) / self.m
        zdd = fz / self.m - self.g
        Xd = vertcat(xd, yd, zd, xdd, ydd, zdd)
        return Xd

    def set_opti_solver(self, solver='ipopt'):
        if solver is 'ipopt':
            options = dict()
            options["print_time"] = False
            options["ipopt"] = {"print_level": 0}
            options["expand"] = True
            self.opti.solver('ipopt', options)

        elif solver is 'sqp':
            options = dict()
            options["qpsol"] = 'qrqp'
            options["qpsol_options"] = {"print_iter": False, "print_header": False}
            options["print_iteration"] = False
            options["print_header"] = False
            options["print_status"] = False
            self.opti.solver('sqpmethod', options)


    def set_opti(self, num_of_phases, segments_per_phase):
        self.num_of_phases = num_of_phases
        self.segments_per_phase = segments_per_phase
        self.knots_per_phase = self.segments_per_phase + 1

        self.num_of_segments = self.num_of_phases * self.segments_per_phase
        self.num_of_knots = self.num_of_phases * self.knots_per_phase

        print('self.num_of_phases      ', self.num_of_phases)
        print('self.segments_per_phase ', self.segments_per_phase)
        print('self.knots_per_phase    ', self.knots_per_phase)
        print('self.num_of_segments    ', self.num_of_segments)
        print('self.num_of_knots       ', self.num_of_knots)


        self.Xdim = 6
        self.Udim = 4

        self.opti = casadi.Opti()  # Optimization problem
        self.set_opti_solver('ipopt')
        # self.set_opti_solver('sqp')

        self.Ts = self.opti.parameter(self.num_of_phases)
        self.dt = self.Ts / self.segments_per_phase  # duration of a control interval

        # ---- variables ---------
        self.X = self.opti.variable(self.Xdim, self.num_of_knots)  # state trajectory
        self.c = self.X[:3, :]
        self.cd = self.X[-3:, :]
        self.x = self.X[0, :]
        self.y = self.X[1, :]
        self.z = self.X[2, :]
        self.xd = self.X[3, :]
        self.yd = self.X[4, :]
        self.zd = self.X[5, :]
        self.U = self.opti.variable(self.Udim, self.num_of_knots)  # control trajectory
        self.px = self.U[0, :]
        self.py = self.U[1, :]
        self.pz = self.U[2, :]
        self.fz = self.U[3, :]

        self.W = self.opti.variable(8, self.num_of_knots)
        self.Wl = self.W[:4, :]
        self.Wr = self.W[-4:, :]

        # body heading
        self.H = self.opti.variable(2, self.num_of_phases)
        self.Hl = self.H[0, :]
        self.Hr = self.H[1, :]


        # footstep positions
        self.Pl = self.opti.variable(3, self.num_of_phases)  # left foot position
        self.Pr = self.opti.variable(3, self.num_of_phases)  # right foot position

        # ---- footstep position constraints ----
        # foot on the ground
        # self.opti.subject_to(self.Pl[2, :] == 0)
        # self.opti.subject_to(self.Pr[2, :] == 0)

        # ini position constraints
        self.Pl0 = self.opti.parameter(3)
        self.Pr0 = self.opti.parameter(3)
        # self.opti.subject_to(self.Pl[:, 0] == self.Pl0)
        # self.opti.subject_to(self.Pr[:, 0] == self.Pr0)

        # phase constriants and kinematic constraints between consecutive steps
        self.dPl_ub = self.opti.parameter(3, self.num_of_phases)
        self.dPl_lb = self.opti.parameter(3, self.num_of_phases)
        self.dPr_ub = self.opti.parameter(3, self.num_of_phases)
        self.dPr_lb = self.opti.parameter(3, self.num_of_phases)
        for i in range(self.num_of_phases):
            if i == 0:
                self.opti.subject_to(self.opti.bounded(self.dPl_lb[:, i], self.Pl[:, i] - self.Pl0, self.dPl_ub[:, i]))
                self.opti.subject_to(self.opti.bounded(self.dPr_lb[:, i], self.Pr[:, i] - self.Pr0, self.dPr_ub[:, i]))
            else:
                self.opti.subject_to(self.opti.bounded(self.dPl_lb[:, i], self.Pl[:, i] - self.Pl[:, i - 1], self.dPl_ub[:, i]))
                self.opti.subject_to(self.opti.bounded(self.dPr_lb[:, i], self.Pr[:, i] - self.Pr[:, i - 1], self.dPr_ub[:, i]))

                self.opti.subject_to(self.opti.bounded(self.dPl_lb[0, i], self.Hl[:, i] - self.Hl[:, i - 1], self.dPl_ub[0, i]))
                self.opti.subject_to(self.opti.bounded(self.dPr_lb[0, i], self.Hr[:, i] - self.Hr[:, i - 1], self.dPr_ub[0, i]))





        # foot self-collision constraints
        for i in range(1, self.num_of_phases-1):
            step_clearence = self.rotz((self.Hl[i]+self.Hr[i])/2).T @ (self.Pl[:, i] - self.Pr[:, i])
            self.opti.subject_to(self.opti.bounded(-0.5, step_clearence[0], 0.5))
            self.opti.subject_to(self.opti.bounded(0.2, step_clearence[1], 0.5))


        # cmd velocity cost: relationship between the different foot in adjacent phases, incremental formulation
        self.cmd_vel = self.opti.parameter(6, 1)
        cmd_vel_linear_cost = 0
        cmd_vel_angular_cost = 0
        for i in range(1, self.num_of_phases):
            delta_Pl = self.Pl[:, i] - self.Pr[:, i - 1]
            delta_Pr = self.Pr[:, i] - self.Pl[:, i - 1]
            delta_Pl_local = self.rotz(self.Hr[i - 1]).T @ delta_Pl
            delta_Pr_local = self.rotz(self.Hl[i - 1]).T @ delta_Pr
            cmd_vel_linear_cost += sumsqr(delta_Pl_local - self.cmd_vel[:3] * self.Ts[i])
            cmd_vel_linear_cost += sumsqr(delta_Pr_local - self.cmd_vel[:3] * self.Ts[i])
            cmd_vel_angular_cost += sumsqr(self.Hl[i] - self.Hr[i - 1] - self.cmd_vel[-1]*self.Ts[i])
            cmd_vel_angular_cost += sumsqr(self.Hr[i] - self.Hl[i - 1] - self.cmd_vel[-1]*self.Ts[i])

        self.opti.subject_to(self.Hl[0] == 0)
        self.opti.subject_to(self.Hr[0] == 0)


        # final footstep pose const

        final_step_clearance = self.rotz(self.Hl[-1]).T@self.Pl[:, -1] - self.rotz(self.Hr[-1]).T@self.Pr[:, -1]
        # self.opti.subject_to(final_step_clearance == np.array([0.0, 0.2, 0.0])) # xyz
        self.opti.subject_to(final_step_clearance[:2] == np.array([0.0, 0.2]))  # xy
        self.opti.subject_to(self.Hl[-1] == self.Hr[-1])


        # ---- objective ---------
        # self.opti.minimize(sumsqr(diff(self.U,1,1)))
        self.opti.minimize(1 * sumsqr(diff(self.U, 1, 1)) +
                           1 * sumsqr(diff(self.W, 1, 1)) +
                           10 * sumsqr(diff(self.H, 1, 1)) +
                           10 * cmd_vel_linear_cost +
                           10 * cmd_vel_angular_cost
                           )

        # ---- dynamics constraints ---------
        self.set_opti_dynamics_constraint_direct_collocation(collocation_scheme='hermite-simpson')
        # self.set_opti_dynamics_constraint_multiple_shooting(method='euler')

        # ---- unilateral constraints ------
        self.opti.subject_to(self.fz > 0)


        # ---- height limit ----
        # self.opti.subject_to(self.z < 1.3)

        # ---- cop constraints ----
        self.Wl_up = self.opti.parameter(self.num_of_phases)
        self.Wr_up = self.opti.parameter(self.num_of_phases)
        for i in range(self.num_of_phases):
            self.opti.subject_to(self.opti.bounded(0, self.Wl[:, i*self.knots_per_phase:(i+1)*self.knots_per_phase], self.Wl_up[i]))
            self.opti.subject_to(self.opti.bounded(0, self.Wr[:, i*self.knots_per_phase:(i+1)*self.knots_per_phase], self.Wr_up[i]))


        # # center of pressure is decided by vertex weights
        for i in range(self.num_of_phases):
            for j in range(self.knots_per_phase):
                k = i*self.knots_per_phase + j

                cop = (self.Pl[:,i] + self.rotz(self.Hl[i])@self.Vs)@self.Wl[:,k] + (self.Pr[:,i] + self.rotz(self.Hr[i])@self.Vs)@self.Wr[:,k]
                self.opti.subject_to(self.px[k] == cop[0])
                self.opti.subject_to(self.py[k] == cop[1])
                self.opti.subject_to(self.pz[k] == cop[2])
                self.opti.subject_to(sum1(self.W[:, k]) == 1)


        # ---- boundary constraints -----
        self.X0 = self.opti.parameter(6, 1)
        self.opti.subject_to(self.X[:, 0] == self.X0)

        # self.XT = self.opti.parameter(6, 1)
        # self.opti.subject_to(self.X[:,-1] == self.XT)
        self.opti.subject_to(self.c[:, -1] == (self.Pl[:, -1] + self.Pr[:, -1]) / 2 + np.array([0, 0, self.norminal_com_height]))
        self.opti.subject_to(self.cd[:, -1] == [0.0, 0.0, 0.0])


    def rotz(self, theta):
        r1 = horzcat(casadi.cos(theta), -casadi.sin(theta), 0)
        r2 = horzcat(casadi.sin(theta), casadi.cos(theta), 0)
        r3 = horzcat(0,0,1)
        Rz = vertcat(r1, r2, r3)
        return Rz

    def euler_integrator(self, X, U, dt):
        X_next = X + 1 / 2 * dt * self.f(X, U)
        return X_next

    def rk4_integrator(self, X, U, dt):
        k1 = self.f(X, U)
        k2 = self.f(X + dt / 2 * k1, U)
        k3 = self.f(X + dt / 2 * k2, U)
        k4 = self.f(X + dt * k3, U)
        X_next = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return X_next

    def set_opti_dynamics_constraint_multiple_shooting(self, method='euler'):
        if method is 'euler':
            for k in range(self.num_of_segments):
                X_next = self.euler_integrator(self.X[:, k], self.U[:, k], self.dt)
                self.opti.subject_to(self.X[:, k + 1] == X_next)
        elif method is 'rk4':
            for k in range(self.num_of_segments):
                X_next = self.rk4_integrator(self.X[:, k], self.U[:, k], self.dt)
                self.opti.subject_to(self.X[:, k + 1] == X_next)

    def set_opti_dynamics_constraint_direct_collocation(self, collocation_scheme='hermite-simpson'):
        if collocation_scheme == 'trapezoidal':
            for i in range(self.num_of_phases):
                for j in range(self.segments_per_phase):
                    k = i * self.knots_per_phase + j
                    self.opti.subject_to(self.X[:, k + 1] - self.X[:, k] == 0.5 * self.dt[i] * (self.f(self.X[:, k + 1], self.U[:, k + 1]) + self.f(self.X[:, k], self.U[:, k])))

        elif collocation_scheme == 'hermite-simpson':
            for i in range(self.num_of_phases):
                for j in range(self.segments_per_phase):
                    k = i * self.knots_per_phase + j
                    # print(i, k, k+1)
                    X_mid = 1 / 2 * (self.X[:, k] + self.X[:, k + 1]) + self.dt[i] / 8 * (self.f(self.X[:, k], self.U[:, k]) - self.f(self.X[:, k + 1], self.U[:, k + 1]))
                    U_mid = 1 / 2 * (self.U[:, k] + self.U[:, k + 1])
                    self.opti.subject_to(self.X[:, k + 1] - self.X[:, k] == 1 / 6 * self.dt[i] * (self.f(self.X[:, k], self.U[:, k]) + 4 * self.f(X_mid, U_mid) + self.f(self.X[:, k + 1], self.U[:, k + 1])))
                # print('connection: ', i * self.knots_per_phase + self.segments_per_phase, i * self.knots_per_phase + self.knots_per_phase)
                if i < self.num_of_phases-1:
                    self.opti.subject_to(self.X[:, i * self.knots_per_phase + self.segments_per_phase] == self.X[:, (i+1) * self.knots_per_phase])


    def set_initial(self, warm_start=True):
        if warm_start and self.solution_saved:
            print('Warm start initialization!')
            self.opti.set_initial(self.X, self.solution.value(self.X))
            self.opti.set_initial(self.U, self.solution.value(self.U))
            self.opti.set_initial(self.W, self.solution.value(self.W))
            # self.opti.set_initial(self.Pl, self.solution.value(self.Pl))
            # self.opti.set_initial(self.Pr, self.solution.value(self.Pr))
        else:
            print('Cold start initialization!')
            self.opti.set_initial(self.z, 1.0)
            self.opti.set_initial(self.fz, self.m * self.g)


    def generate_support_indexes(self, cur_com_vel, num_of_phases, ini_DS=True, fin_DS=True, plot=False):

        if cur_com_vel[1] == 0:
            ini_support = 1
        else:
            ini_support = np.sign(cur_com_vel[1])  # 1 for LS, -1 for RS


        if ini_DS:
            if fin_DS:
                support_indexes = np.concatenate(([0], ini_support * (-1) ** np.arange(num_of_phases - 2), [0]))
            else:
                support_indexes = np.concatenate(([0], ini_support * (-1)**np.arange(num_of_phases-1)))
        else:
            if fin_DS:
                support_indexes = np.concatenate((ini_support * (-1) ** np.arange(num_of_phases - 1), [0]))
            else:
                support_indexes = ini_support * (-1) ** np.arange(num_of_phases)

        if plot is True:
            print('ini_DS, fin_DS', ini_DS, fin_DS)
            print('cur_com_vel[1]', cur_com_vel[1])
            print('ini_support', ini_support)
            print('support_indexes: ', support_indexes)


        return support_indexes


    def generate_support_durations(self, support_indexes):

        support_durations = []
        for support_index in support_indexes:
            if support_index == 0:
                support_durations.append(self.double_support_duration)
            elif support_index == 1 or support_index == -1:
                support_durations.append(self.single_support_duration)
            else:
                print('[Error] support_index must be 1 or 0 or -1!')

        return np.array(support_durations)


    def plan(self, cur_com_pos, cur_com_vel, cur_lfoot_pos, cur_rfoot_pos, cmd_vel=np.array([0,0,0]), ini_DS=True, fin_DS=True, support_indexes=False, support_durations=False, plot=False):

        if support_indexes == False:
            self.support_indexes = self.generate_support_indexes(cur_com_vel, self.num_of_phases, ini_DS=ini_DS, fin_DS=fin_DS)
        else:
            self.support_indexes = support_indexes

        if support_durations == False:
            self.support_durations = self.generate_support_durations(self.support_indexes)
        else:
            self.support_durations = support_durations


        X0 = np.concatenate((cur_com_pos, cur_com_vel))

        # set Wl_max and Wr_max based on support_indexes
        for phase_index, support_index in enumerate(self.support_indexes):
            if support_index == 0:  # DS
                self.opti.set_value(self.Wl_up[phase_index], 1.0)
                self.opti.set_value(self.Wr_up[phase_index], 1.0)
            elif support_index == 1:  # LS
                self.opti.set_value(self.Wl_up[phase_index], 1.0)
                self.opti.set_value(self.Wr_up[phase_index], 0.0)
            elif support_index == -1:  # RS
                self.opti.set_value(self.Wl_up[phase_index], 0.0)
                self.opti.set_value(self.Wr_up[phase_index], 1.0)



        # set dPl_ub and dPr_ub based on support indexes
        for phase_index in range(self.num_of_phases):
            support_index = self.support_indexes[phase_index]

            if support_index == 0:  # DS
                self.opti.set_value(self.dPl_ub[:, phase_index], np.array([0.0, 0.0, 0.0]))
                self.opti.set_value(self.dPl_lb[:, phase_index], np.array([0.0, 0.0, 0.0]))
                self.opti.set_value(self.dPr_ub[:, phase_index], np.array([0.0, 0.0, 0.0]))
                self.opti.set_value(self.dPr_lb[:, phase_index], np.array([0.0, 0.0, 0.0]))
                # self.opti.set_value(self.step_width_ub[:, phase_index], 1.0)
                # self.opti.set_value(self.step_width_lb[:, phase_index], 0.2)
            elif support_index == 1:  # LS
                self.opti.set_value(self.dPl_ub[:, phase_index], np.array([0.0, 0.0, 0.0]))
                self.opti.set_value(self.dPl_lb[:, phase_index], np.array([0.0, 0.0, 0.0]))
                self.opti.set_value(self.dPr_ub[:, phase_index], np.array([1.0, 1.0, 1.0]))
                self.opti.set_value(self.dPr_lb[:, phase_index], np.array([-1.0, -1.0, -1.0]))

            elif support_index == -1:  # RS
                self.opti.set_value(self.dPl_ub[:, phase_index], np.array([1.0, 1.0, 1.0]))
                self.opti.set_value(self.dPl_lb[:, phase_index], np.array([-1.0, -1.0, -1.0]))
                self.opti.set_value(self.dPr_ub[:, phase_index], np.array([0.0, 0.0, 0.0]))
                self.opti.set_value(self.dPr_lb[:, phase_index], np.array([0.0, 0.0, 0.0]))

        self.opti.set_value(self.Ts, self.support_durations)
        self.opti.set_value(self.X0, X0)
        # self.opti.set_value(self.XT, XT)
        self.opti.set_value(self.Pl0, cur_lfoot_pos)#[cur_lfoot_pos[0], cur_lfoot_pos[1], height_map(cur_lfoot_pos[0], cur_lfoot_pos[1])])
        self.opti.set_value(self.Pr0, cur_rfoot_pos)#[cur_rfoot_pos[0], cur_rfoot_pos[1], height_map(cur_rfoot_pos[0], cur_rfoot_pos[1])])

        self.opti.set_value(self.cmd_vel, cmd_vel)

        self.set_initial(warm_start=True)
        t = time.time()
        self.solution = self.opti.solve()
        solve_time = time.time() - t
        print('solve_time:', solve_time)
        self.solution_saved = True

        self.post_process(printout=True, plot=plot, sparsity=False)

        return self.return_motion_plan()

    def return_heading_angle(self):
        return self.solution.value(self.Hl), self.solution.value(self.Hr)

    def get_time(self):
        Ts = self.solution.value(self.Ts)
        T_cum = np.cumsum(np.insert(Ts,0,0))
        time = np.array([np.linspace(T_cum[i], T_cum[i+1], self.knots_per_phase) for i in range(len(T_cum)-1)]).flatten()
        # time = np.unique(time)
        return Ts, T_cum, time

    def get_state(self):
        return self.solution.value(self.X)

    def get_input(self):
        return self.solution.value(self.U)


    def return_motion_plan(self):
        Ts, T_cum, time = self.get_time()
        com_pos = self.solution.value(self.c)
        com_vel = self.solution.value(self.cd)
        com_trajectory = CoMTrajectory(time, com_pos, com_vel, np.zeros_like(com_pos), debug_plot=False)
        left_foot_positions = self.solution.value(self.Pl).T
        right_foot_positions = self.solution.value(self.Pr).T
        return (com_trajectory, left_foot_positions, right_foot_positions, self.support_indexes, self.support_durations)


    def post_process(self, printout=True, plot=True, sparsity=True):
        if printout is True:
            print('Pl\n', self.solution.value(self.Pl).T)
            print('Pr\n', self.solution.value(self.Pr).T)
        if plot is True:
            self.plot_state_and_input(self.solution)
            self.plot_footstep(self.solution.value(self.X[:3,:]), self.solution.value(self.U), self.solution.value(self.Pl), self.solution.value(self.Pr), self.support_indexes)
            plt.show()
        if sparsity is True:
            self.sparsity(self.solution)


    def sparsity(self, solution):
        import matplotlib.pyplot as plt
        plt.figure()
        plt.title('Hessian')
        plt.spy(solution.value(hessian(self.opti.f, self.opti.x)[0]))
        plt.title('Jacobian')
        plt.figure()
        plt.spy(solution.value(jacobian(self.opti.g, self.opti.x)))
        plt.show()

    def plot_footstep(self, com, zmp, pl, pr, support_indexes):

        n = len(support_indexes)
        size_norm = 10
        fig, axs = plt.subplots(nrows=1, ncols=n, figsize=(size_norm*n, size_norm))


        foot_size = (0.2, 0.1)
        for i, support_index in enumerate(support_indexes):
            if support_index == 0:
                for j in range(i,n):
                    lfoot_pos = pl[:, i]
                    rfoot_pos = pr[:, i]
                    lfoot_rect = plt.Rectangle((lfoot_pos[0] - foot_size[0] / 2, lfoot_pos[1] - foot_size[1] / 2), foot_size[0], foot_size[1], facecolor=(0.83203125, 0.90625, 0.828125, 1.0), edgecolor=(0.51 , 0.702, 0.4, 1.0), linewidth=1.5)
                    rfoot_rect = plt.Rectangle((rfoot_pos[0] - foot_size[0] / 2, rfoot_pos[1] - foot_size[1] / 2), foot_size[0], foot_size[1], facecolor=(0.8515625, 0.90625, 0.984375, 1.0), edgecolor=(0.424, 0.557, 0.749, 1.0), linewidth=1.5)
                    axs[j].add_patch(lfoot_rect)
                    axs[j].add_patch(rfoot_rect)
            elif support_index == 1:
                for j in range(i,n):
                    lfoot_pos = pl[:, i]
                    # lfoot_rect = plt.Rectangle((lfoot_pos[0] - foot_size[0] / 2, lfoot_pos[1] - foot_size[1] / 2), foot_size[0], foot_size[1], color=(0.83203125, 0.90625, 0.828125, 1.0), fill=True)
                    lfoot_rect = plt.Rectangle((lfoot_pos[0] - foot_size[0] / 2, lfoot_pos[1] - foot_size[1] / 2), foot_size[0], foot_size[1], facecolor=(0.83203125, 0.90625, 0.828125, 1.0), edgecolor=(0.51 , 0.702, 0.4, 1.0), linewidth=1.5)
                    axs[j].add_patch(lfoot_rect)
            elif support_index == -1:
                for j in range(i,n):
                    rfoot_pos = pr[:, i]
                    rfoot_rect = plt.Rectangle((rfoot_pos[0] - foot_size[0] / 2, rfoot_pos[1] - foot_size[1] / 2), foot_size[0], foot_size[1], facecolor=(0.8515625, 0.90625, 0.984375, 1.0), edgecolor=(0.424, 0.557, 0.749, 1.0), linewidth=1.5)
                    axs[j].add_patch(rfoot_rect)


        titles = ['$DS$', '$LS$','$DS$', '$RS$', '$DS$']
        for i in range(n):
            # axs[i].title.set_text('step '+str(i))
            axs[i].title.set_text(titles[i])
            axs[i].plot(com[0,0:self.knots_per_phase*(i+1)], com[1,0:self.knots_per_phase*(i+1)], '.')
            axs[i].plot(zmp[0,0:self.knots_per_phase*(i+1)], zmp[1,0:self.knots_per_phase*(i+1)], 'r.')
            axs[i].legend(['$CoM$', '$CoP$'])

            axs[i].axhline(y=0, color="red", linestyle="--")
            axs[i].axvline(x=0, color="green", linestyle="--")


            axs[i].set_aspect('equal', 'datalim')
            axs[i].grid()

        # save plot as pdf
        # plt.savefig('footstep_optimization.pdf', bbox_inches='tight')

    def plot_state_and_input(self, solution):
        import matplotlib
        import matplotlib.pyplot as plt
        matplotlib.rcParams['text.usetex'] = True
        matplotlib.rcParams['text.latex.preamble'] = [r"\usepackage{amsmath}"]

        Ts, T_cum, time = self.get_time()

        X = solution.value(self.X)
        U = solution.value(self.U)

        figsize = (16, 9)
        legend_loc = 'upper center'
        X_legend = ['$x$', '$y$', '$z$', '$\dot{x}$', '$\dot{y}$', '$\dot{z}$']
        U_legend = ['$p_x$', '$p_y$', '$p_z$', '$f_z$']

        nrows, ncols = 4, 3
        fig, axs = plt.subplots(nrows, ncols, figsize=figsize)
        for r in range(nrows):
            for c in range(ncols):
                i = r * ncols + c
                for T in T_cum:
                    axs[r][c].axvline(x=T, linestyle="--", color="k")
                if r < 2:
                    axs[r][c].plot(time, X[i, :], '.-')
                    axs[r][c].legend([X_legend[i]],loc=legend_loc)
                    # if r == 0 and c < 2:
                    #     axs[r][c].plot(time, U[c, :], 'r.-')
                elif r == 2:
                    axs[r][c].plot(time, U[c, :], 'r.-')
                    axs[r][c].legend([U_legend[c]],loc=legend_loc)
                elif r == 3:
                    if c < 2:
                        axs[r][c].plot(time, X[c, :], '.-')
                        axs[r][c].plot(time, U[c, :], 'r.-')
                    elif c == 2:
                        axs[r][c].plot(time, U[c+1, :], 'r.-')
                        axs[r][c].legend([U_legend[c+1]],loc=legend_loc)


        figsize = (10, 10)
        legend_loc = 'upper right'
        Wl, Wr = solution.value(self.Wl), solution.value(self.Wr)
        Wl_legend = ['$w_{L,1}$', '$w_{L,2}$', '$w_{L,3}$', '$w_{L,4}$']
        Wr_legend = ['$w_{R,1}$', '$w_{R,2}$', '$w_{R,3}$', '$w_{R,4}$']
        nrows, ncols = 2, 1
        fig, axs = plt.subplots(nrows, ncols, figsize=figsize)
        for r in range(nrows):
            for c in range(ncols):
                i = r * ncols + c
                if r == 0:
                    axs[r].plot(time, Wl.T, '.-')
                    axs[r].legend(Wl_legend,loc=legend_loc)
                    axs[r].title.set_text(r'$\boldsymbol{w}_L$')
                    axs[r].set_xlabel(r'$t$')
                    axs[r].axhline(y=0, color="red", linestyle="--")
                    axs[r].axhline(y=1, color="red", linestyle="--")
                    for T in T_cum:
                        axs[r].axvline(x=T,linestyle="--")
                elif r == 1:

                    axs[r].plot(time, Wr.T, '.-')
                    axs[r].legend(Wr_legend,loc=legend_loc)
                    axs[r].title.set_text(r'$\boldsymbol{w}_R$')
                    axs[r].set_xlabel(r'$t$')
                    axs[r].axhline(y=0, color="red", linestyle="--")
                    axs[r].axhline(y=1, color="red", linestyle="--")
                    for T in T_cum:
                        axs[r].axvline(x=T,linestyle="--")

        # save plot as pdf
        # plt.savefig('vertex_weightings.pdf', bbox_inches='tight')







if __name__ == "__main__":

    red, green, blue = (1,0,0), (0,1,0), (0,0,1)

    import pybullet, pybullet_data
    pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF('plane.urdf', basePosition=[0,0,-0.001])
    pybullet.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-90, cameraPitch=-60, cameraTargetPosition=[0,0,0.5])
    pybullet.setRealTimeSimulation(True)



    support_indexes = [0,1,0,-1,0,1,0,-1,0]
    num_of_phases = len(support_indexes)

    robot_param = {'m':1,
                   'g':9.81,
                   'foot_length':0.2,
                   'foot_width':0.1,
                   'norminal_com_height':0.8,
                   'single_support_duration': 0.8,
                   'double_support_duration': 0.2}
    com_motion_planner = ComMotionPlanner(num_of_phases=num_of_phases, robot_param=robot_param, segments_per_phase=10,)


    com_motion_planner.plan(cur_com_pos=[0.0, 0.0, 0.8],
                            cur_com_vel=[0.0, 0.0, 0.0],
                            cur_lfoot_pos=[0, 0.1, 0.0],
                            cur_rfoot_pos=[0, -0.1, 0],
                            cmd_vel=np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
                            support_indexes=support_indexes,
                            plot=False)

    com_trajectory, left_foot_positions, right_foot_positions, support_indexes, support_durations = com_motion_planner.return_motion_plan()
    Hl, Hr = com_motion_planner.return_heading_angle()


    # plot footsteps
    import utils.transformations as tf
    for pl, pr, support_index, hl, hr in zip(left_foot_positions, right_foot_positions, support_indexes, Hl, Hr):
        addDebugRectangle(position=pl, quaternion=tf.quaternion_from_euler(0, 0, hl), color=green)
        addDebugRectangle(position=pr, quaternion=tf.quaternion_from_euler(0, 0, hr), color=blue)


    # plot com trajectory
    addDebugTrajectory3D(com_trajectory.pos, color=red)
    addDebugTrajectory2D(com_trajectory.pos, color=red)


    while (pybullet.isConnected()):
        pass







