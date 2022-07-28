# Example agent.
from typing import Tuple, List

import numpy as np
import math
from scipy.integrate import ode
import numpy.linalg as la

from dryvr_plus_plus.agents.base_agent import BaseAgent
from dryvr_plus_plus.map.lane_map import LaneMap
from dynamics_forward_euler import simulate

class quadrotor_agent2(BaseAgent):
    def __init__(self, id, code=None, file_name=None):
        super().__init__(id, code, file_name)

    def TC_simulate(self, mode: List[str], initialCondition, time_bound, time_step, lane_map: LaneMap = None) -> np.ndarray:
        tmp = simulate(0, time_bound, time_step, initialCondition)
        res = tmp[0]
        return res

class quadrotor_agent(BaseAgent):
    def __init__(self, id, code=None, file_name=None):
        # Calling the constructor of tha base class
        super().__init__(id, code, file_name)
        # step0-1: start to do some initialization for system params and L1 inputs
        self.m  = 0.752
        self.g = 9.81
        self.J = 1e-3*np.diag([2.5, 2.1, 4.3])
        self.e3 = np.array([0.,0.,1.])

        """ L1-related parameters """
        self.As_v = -5.0 # parameter for L1
        self.As_omega = -5.0 # parameter for L1
        self.dt_L1 = 1/20 # sample time for L1 AC, for simplicity, set the same as the simulation step size

        """ For large uncertainties ..."""
        self.ctoffq1Thrust = 5*7 # cutoff frequency for thrust channel LPF (rad/s)
        self.ctoffq1Moment = 1*7 # cutoff frequency for moment channels LPF1 (rad/s)
        self.ctoffq2Moment = 1*7 # cutoff frequency for moment channels LPF2 (rad/s)

        self.L1_params = (self.As_v, self.As_omega, self.dt_L1, self.ctoffq1Thrust, self.ctoffq1Moment, self.ctoffq2Moment, self.m, self.g, self.J )

        """ Geometric control gains """
        self.kx = 16*self.m*np.ones((3,)) # position gains
        self.kv = 5.6*self.m*np.ones((3,)) # velocity gains
        self.kR = 8.81*np.ones((3,)) # angular gains
        self.kW = 2.54*np.ones((3,)) # rotational velocity gains

        """ Initialization of L1 inputs """
        self.v_hat_prev = np.array([0.0, 0.0, 0.0])
        self.omega_hat_prev = np.array([0.0, 0.0, 0.0])
        self.R_prev = np.zeros((9,)).reshape(3,3)
        self.v_prev = np.array([0.0,0.0,0.0])
        self.omega_prev = np.array([0.0,0.0,0.0])

        self.u_b_prev = np.array([0.0,0.0,0.0,0.0])
        self.u_ad_prev = np.array([0.0,0.0,0.0,0.0])
        self.sigma_m_hat_prev = np.array([0.0,0.0,0.0,0.0])
        self.sigma_um_hat_prev = np.array([0.0,0.0])
        self.lpf1_prev = np.array([0.0,0.0,0.0,0.0])
        self.lpf2_prev = np.array([0.0,0.0,0.0,0.0])

        self.din_L1 = (self.v_hat_prev, self.omega_hat_prev, self.R_prev, self.v_prev, self.omega_prev,
                       self.u_b_prev, self.u_ad_prev, self.sigma_m_hat_prev, self.sigma_um_hat_prev, 
                       self.lpf1_prev, self.lpf2_prev)


    def L1_AC(self, R, W, x, v, f, M):

        (As_v, As_omega, dt, ctoffq1Thrust, ctoffq1Moment, ctoffq2Moment, kg_vehicleMass, GRAVITY_MAGNITUDE, J ) = self.L1_params
        (v_hat_prev, omega_hat_prev, R_prev, v_prev, omega_prev,
        u_b_prev, u_ad_prev, sigma_m_hat_prev, sigma_um_hat_prev,
        lpf1_prev, lpf2_prev) = self.din_L1

        # == begin L1 adaptive control ==
        # first do the state predictor
        # load translational velocity
        v_now = v

        # load rotational velocity
        omega_now = W

        massInverse = 1.0 / kg_vehicleMass

        # compute prediction error (on previous step)
        vpred_error_prev = v_hat_prev - v_prev # computes v_tilde for (k-1) step
        omegapred_error_prev = omega_hat_prev - omega_prev # computes omega_tilde for (k-1) step

        v_hat = v_hat_prev + (self.e3 * GRAVITY_MAGNITUDE - R_prev[:,2]* (u_b_prev[0] + u_ad_prev[0] + sigma_m_hat_prev[0]) * massInverse + R_prev[:,0] * sigma_um_hat_prev[0] * massInverse + R_prev[:,1] * sigma_um_hat_prev[1] * massInverse + vpred_error_prev * As_v) * dt
        Jinv = la.inv(J)
        # temp vector: thrustMomentCmd[1--3] + u_ad_prev[1--3] + sigma_m_hat_prev[1--3]
        # original form
        tempVec = np.array([u_b_prev[1] + u_ad_prev[1] + sigma_m_hat_prev[1], u_b_prev[2] + u_ad_prev[2] + sigma_m_hat_prev[2], u_b_prev[3] + u_ad_prev[3] + sigma_m_hat_prev[3]])
        omega_hat = omega_hat_prev + (-np.matmul(Jinv, np.cross(omega_prev, (np.matmul(J, omega_prev)))) + np.matmul(Jinv, tempVec) + omegapred_error_prev * As_omega) * dt

        # update the state prediction storage
        v_hat_prev = v_hat
        omega_hat_prev = omega_hat

        # compute prediction error (for this step)
        vpred_error = v_hat - v_now
        omegapred_error = omega_hat - omega_now

        # exponential coefficients coefficient for As
        exp_As_v_dt = math.exp(As_v * dt)
        exp_As_omega_dt = math.exp(As_omega * dt)

        # latter part of uncertainty estimation (piecewise constant) (step2: adaptation law)
        PhiInvmu_v = vpred_error / (exp_As_v_dt - 1) * As_v * exp_As_v_dt
        PhiInvmu_omega = omegapred_error / (exp_As_omega_dt - 1) * As_omega * exp_As_omega_dt

        sigma_m_hat = np.array([0.0,0.0,0.0,0.0]) # estimated matched uncertainty
        sigma_m_hat_2to4 = np.array([0.0,0.0,0.0]) # second to fourth element of the estimated matched uncertainty
        sigma_um_hat = np.array([0.0,0.0]) # estimated unmatched uncertainty

        sigma_m_hat[0] = np.dot(R[:,2], PhiInvmu_v) * kg_vehicleMass
        # turn np.dot(R[:,2], PhiInvmu_v) * kg_vehicleMass to -np.dot(R[:,2], PhiInvmu_v) * kg_vehicleMass
        sigma_m_hat_2to4 = -np.matmul(J, PhiInvmu_omega)
        sigma_m_hat[1] = sigma_m_hat_2to4[0]
        sigma_m_hat[2] = sigma_m_hat_2to4[1]
        sigma_m_hat[3] = sigma_m_hat_2to4[2]

        sigma_um_hat[0] = -np.dot(R[:,0], PhiInvmu_v) * kg_vehicleMass
        sigma_um_hat[1] = -np.dot(R[:,1], PhiInvmu_v) * kg_vehicleMass

        # store uncertainty estimations
        sigma_m_hat_prev = sigma_m_hat
        sigma_um_hat_prev = sigma_um_hat

        # compute lpf1 coefficients
        lpf1_coefficientThrust1 = math.exp(- ctoffq1Thrust * dt)
        lpf1_coefficientThrust2 = 1.0 - lpf1_coefficientThrust1

        lpf1_coefficientMoment1 = math.exp(- ctoffq1Moment * dt)
        lpf1_coefficientMoment2 = 1.0 - lpf1_coefficientMoment1

        # update the adaptive control
        u_ad_int = np.array([0.0,0.0,0.0,0.0])
        u_ad = np.array([0.0,0.0,0.0,0.0])

        # low-pass filter 1 (negation is added to u_ad_prev to filter the correct signal)
        u_ad_int[0] = lpf1_coefficientThrust1 * (lpf1_prev[0]) + lpf1_coefficientThrust2 * sigma_m_hat[0]
        u_ad_int[1:3] = lpf1_coefficientMoment1 * (lpf1_prev[1:3]) + lpf1_coefficientMoment2 * sigma_m_hat[1:3]

        lpf1_prev = u_ad_int # store the current state

        # coefficients for the second LPF on the moment channel
        lpf2_coefficientMoment1 = math.exp(- ctoffq2Moment * dt)
        lpf2_coefficientMoment2 = 1.0 - lpf2_coefficientMoment1

        # low-pass filter 2 (optional)
        u_ad[0] = u_ad_int[0] # only one filter on the thrust channel
        u_ad[1:3] = lpf2_coefficientMoment1 * lpf2_prev[1:3] + lpf2_coefficientMoment2 * u_ad_int[1:3]

        lpf2_prev = u_ad # store the current state

        u_ad = -u_ad

        # store the values for next iteration (negation is added to u_ad_prev to filter the correct signal)
        u_ad_prev = u_ad

        v_prev = v_now
        omega_prev = omega_now
        R_prev = R
        u_b_prev = np.array([f,M[0],M[1],M[2]])

        controlcmd_L1 = np.array([f,M[0],M[1],M[2]]) + u_ad_prev

        self.din_L1 = (v_hat_prev, omega_hat_prev, R_prev, v_prev, omega_prev,
        u_b_prev, u_ad_prev, sigma_m_hat_prev, sigma_um_hat_prev,
        lpf1_prev, lpf2_prev)

        f_L1 = controlcmd_L1[0]
        M_L1 = controlcmd_L1[1:4]
        return (f_L1, M_L1, sigma_m_hat)

    def geometric_control(self, t, R, W, x, v, d_in):

        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = self.position_errors( x, xd, v, xd_dot)

        f = np.dot(self.kx*ex + self.kv*ev + self.m*self.g*self.e3 - self.m*xd_2dot, R.dot(self.e3) )
        W_hat = self.hat(W)

        R_dot = R.dot(W_hat)
        x_2dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        ex_2dot = x_2dot - xd_2dot

        f_dot = ( self.kx*ev + self.kv*ex_2dot - self.m*xd_3dot).dot(R.dot(self.e3)) + ( self.kx*ex + self.kv*ev + self.m*self.g*self.e3 - self.m*xd_2dot).dot(np.dot(R_dot,self.e3))

        x_3dot = -1/self.m*( f_dot*R + f*R_dot ).dot(self.e3)
        ex_3dot = x_3dot - xd_3dot

        A = -self.kx*ex - self.kv*ev - self.m*self.g*self.e3 + self.m*xd_2dot
        A_dot = -self.kx*ev - self.kv*ex_2dot + self.m*xd_3dot
        A_2dot = -self.kx*ex_2dot - self.kv*ex_3dot + self.m*xd_4dot

        (Rd, Wd, Wd_dot) = self.get_Rc(A, A_dot, A_2dot , b1d, b1d_dot, b1d_ddot)

        (eR, eW) = self.attitude_errors( R, Rd, W, Wd )
        M= -self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W)) - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd))) - R.T.dot(Rd.dot(Wd_dot)))
        return (f, M, Rd)

    


    # def dynamic(t, state):
    #     x, y = state
    #     x = float(x)
    #     y = float(y)
    #     x_dot = y
    #     y_dot = (1-x**2)*y - x
    #     return [x_dot, y_dot]
    def dynamic(self, t, state, sim_mass):
        x = state[:3] # position
        v = state[3:6] # velocity
        R = np.reshape(state[6:15], (3,3)) # rotation matrix from body to inertial
        W = state[15:18] # angular velocity
        mass = state[18:]

        xd = np.array([2*(1-np.cos(t)), 2*np.sin(t), 1.0 + np.sin(t)]) #desired position
        xd_dot = np.array([2*np.sin(t), 2*np.cos(t), np.cos(t)]) #desired velocity
        xd_ddot = np.array([2*np.cos(t), -2*np.sin(t), -np.sin(t)]) #desired acceleration
        xd_dddot= np.array([-2*np.sin(t), -2*np.cos(t), -np.cos(t)]) #desired jerk
        xd_ddddot = np.array([-2*np.cos(t), 2*np.sin(t), np.sin(t)]) #desired snap

        b1d = np.array([1., 0., 0.]) #desired orientation vector (bint in Algorithm 1)
        b1d_dot=np.array([0., 0., 0.]) #(bint_dot)
        b1d_ddot=np.array([0., 0., 0.]) #(bint_ddot)

        Rd = np.eye(3)
        Wd = np.array([0.,0.,0.])
        Wd_dot = np.array([0.,0.,0.])

        f = np.array([0])
        M = np.array([0,0,0])

        d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                    b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
        (f, M, Rd) = self.geometric_control(t, R, W, x, v, d_in) # computes the control commands
        # print(f)
        (f_L1, M_L1, sigma_m_hat) = self.L1_AC(R, W, x, v, f, M)
        # print(f_L1)
        # print('------------------------')
        # print(M_L1)
 


        """ Uncertain dynamics (use geometric + L1 adaptive control) """
        # sigma_m_thrust = 1.4*math.sin(0.5*(t-5)) + 3.7*math.sin(0.75*(t-5))
        # sigma_m_roll = 0.9*math.sin(0.75*(t-5))
        # sigma_m_pitch = 0.85*(math.sin(t-5) + math.sin(0.5*(t-5)))
        sigma_m_thrust = 0.0
        sigma_m_roll = 0.0
        sigma_m_pitch = 0.0 # tuning knobs (uncertainty/disturbances on the control channel)

        f = f_L1 + sigma_m_thrust # L1 + geometric control 
        M = M_L1 + np.array([sigma_m_roll, sigma_m_pitch, 0.0]) # L1 + geometric control

        x_dot = v
        v_dot = self.g*self.e3 - f/mass*R.dot(self.e3)
        R_dot = np.dot(R, self.hat(W))
        W_dot = np.dot(la.inv(self.J), M - np.cross(W, np.dot(self.J, W)))
        mass_dot = np.zeros(1,)
        X_dot = np.concatenate((x_dot, v_dot, R_dot.flatten(), W_dot, mass_dot))

        return X_dot

    """ Some utility functions """
    def position_errors( self, x, xd, v, vd):
        ex = x - xd
        ev = v - vd
        return (ex, ev)

    def attitude_errors( self, R, Rd, W, Wd ):
        eR = 0.5*self.vee(Rd.T.dot(R) - R.T.dot(Rd))
        eW = W - R.T.dot(Rd.dot(Wd))
        return (eR, eW)

    def vee(self, M):
        return np.array([M[2,1], M[0,2], M[1,0]])

    def hat(self, x):
        hat_x = [0, -x[2], x[1],
                x[2], 0, -x[0],
                -x[1], x[0], 0]
        return np.reshape(hat_x,(3,3))

    def get_Rc(self, A, A_dot, A_2dot, b1d, b1d_dot, b1d_ddot):

        norm_A = la.norm(A)
        b3c = - A/norm_A
        b3c_dot = - A_dot/norm_A + ( np.dot(A, A_dot)*A )/norm_A**3 #eq (4)
        b3c_2dot = - A_2dot/norm_A + ( 2*np.dot(A*A_dot,A_dot) )/norm_A**3 + np.dot( A_dot* A_dot + A*A_2dot ,A)/norm_A**3 - 3*np.dot((A*A_dot)**2,A)/norm_A**5 #eq (7)

        b_ = np.cross(b3c, b1d)
        b_norm = la.norm(b_)
        b_dot = np.cross(b3c_dot, b1d) + np.cross(b3c, b1d_dot)
        b_2dot = np.cross(b3c_2dot, b1d) + 2*np.cross(b3c_dot, b1d_dot) + np.cross(b3c, b1d_ddot)

        b1c =  - np.cross( b3c, b_ )/b_norm
        """b1c = b2c x b3c, equivalently, b2c = b3c x b1c"""
        b1c_dot = ( np.cross(b3c_dot, b_) - np.cross(b3c, b_dot) )/b_norm - np.cross(b3c, b_)*(b_dot* b_)/b_norm**3

        # intermediate steps to calculate b1c_2dot
        m_1 = ( np.cross(b3c_2dot, b_) + 2*np.cross(b3c_dot, b_dot) + np.cross(b3c, b_2dot) )/b_norm
        m_2 = ( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_)/b_norm**3
        m_dot = m_1 - m_2
        n_1 = np.cross(b3c, b_)*np.dot(b_dot, b_)
        n_1dot = ( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_) + np.cross(b3c, b_)*( np.dot(b_2dot, b_)+np.dot(b_dot, b_dot) )
        n_dot = n_1dot/b_norm**3 - 3*n_1*np.dot(b_dot, b_)/b_norm**5
        b1c_2dot = (-m_dot + n_dot)

        Rc = np.reshape([b1c, np.cross(b3c, b1c), b3c],(3,3)).T
        Rc_dot = np.reshape([b1c_dot, ( np.cross(b3c_dot, b1c) + np.cross(b3c, b1c_dot) ), b3c_dot],(3,3)).T
        Rc_2dot = np.reshape( [b1c_2dot, ( np.cross(b3c_2dot, b1c) + np.cross(b3c_dot, b1c_dot) + np.cross(b3c_dot, b1c_dot) + np.cross(b3c, b1c_2dot) ), b3c_2dot],(3,3)).T
        Wc = self.vee(Rc.T.dot(Rc_dot))
        Wc_dot= self.vee( Rc_dot.T.dot(Rc_dot) + Rc.T.dot(Rc_2dot))
        return (Rc, Wc, Wc_dot)



    def TC_simulate(self, mode: List[str], initialCondition, time_bound, time_step, lane_map: LaneMap = None) -> np.ndarray:

        solver = ode(self.dynamic)
        solver.set_integrator('dopri5', nsteps=2000).set_initial_value(initialCondition, 0).set_f_params(self.m*1.0) #tuning knobs (change of simulation mass)
        dt = time_step

        # sim = []

        # xd = []
        # xd_dot = []
        # sigma_m_inj = []
        # sigma_m_hat = []
        # Rd = []
        # command_val = []
        trace = [[0]+initialCondition]
        while solver.successful() and solver.t < time_bound:
            print(solver.t)
            res = solver.integrate(solver.t+dt)
            init = res.flatten().tolist()
            trace.append([solver.t] + init)
            # sim.append(solver.y)
            # xd.append(uav_t.xd)
            # xd_dot.append(uav_t.xd_dot)
            # sigma_m_inj.append(uav_t.sigma_m_inj)
            # sigma_m_hat.append(uav_t.sigma_m_hat)
            # Rd.append(uav_t.Rd)
            # command_val.append(uav_t.command)

            # time_bound = float(time_bound)
            # number_points = int(np.ceil(time_bound/time_step))
            # t = [round(i*time_step, 10) for i in range(0, number_points)]
            # # note: digit of time
            # init = initialCondition
            # trace = [[0]+init]
            # for i in range(len(t)):
            #     # print(init)
            #     r = ode(self.dynamic)
            #     r.set_initial_value(init)
            #     res: np.ndarray = r.integrate(r.t + time_step)
            #     init = res.flatten().tolist()
            #     trace.append([t[i] + time_step] + init)

        # print(trace)
        return np.array(trace)

if __name__ == '__main__':
    aquad = quadrotor_agent2('agent2', file_name='./quad_controller.py')

    # Initialize simulation states
    trace = aquad.TC_simulate(None, [0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,], 10, 0.001)
    # print(trace)
    import matplotlib.pyplot as plt
    plt.plot(trace[:,1], trace[:,2],'b')
    plt.show()