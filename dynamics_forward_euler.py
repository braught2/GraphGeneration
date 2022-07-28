import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as Rot
from scipy.linalg import expm 
# from sympy import OmegaPower
from spatialmath.base import *
from numpy import linalg as LA
# from torch import DoubleTensor
# reference: https://towardsdatascience.com/demystifying-drone-dynamics-ee98b1ba882f
# reference: https://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/
# reference: https://github.com/brianwade1/quadcopter_with_PID_controller/blob/master/Quadcopter_main.py


def f_value_uncertain(J, m, t, state, u):
    #Dynamics
    # state: [p(3), v(3), R_Matrix(9), omega(3)]
    ## p(3)-- x,y,z are position of drone in world/inertial frame
    ## v(3)-- vx,vy,vz are positional/linear velocities in world/inertial frame

    ## omega(3)-- rotational angular velocity in the body frame (from IMU)

    # control: [f, moments(3)]
    ## moments(3)-- the moments in the body frame
    g = 9.81
    vx = state[3]
    vy = state[4]
    vz = state[5]
    Rvec = state[6:15]
    Omega = state[15:18]  # angular velocity vector in body frame

    sigma_m_thrust = 0.4*math.sin(0.5*(t-5)) + 0.7*math.sin(0.75*(t-5))
    sigma_m_roll = 0.6*math.sin(0.75*(t-5))
    sigma_m_pitch = 0.25*(math.sin(t-5) + math.sin(0.5*(t-5)))
    f_thrust = u[0] + sigma_m_thrust
    M = u[1:4] + np.array([sigma_m_roll, sigma_m_pitch, 0.0])

    uncertain_inj = np.array([sigma_m_thrust, sigma_m_roll, sigma_m_pitch, 0 ])

    r_mat = Rvec.reshape(3,3)
    e3 = np.array([0,0,1])

    thrust_proj = f_thrust/m*np.matmul(r_mat,e3)

    Rdot = np.matmul(r_mat,hatOperator(Omega))
    # # angular velocity in the inertial frame
    # Thetadot = omega2thetadot(Omega, Theta)
    # angular acceleration in body frame
    Omega_dot = np.matmul(LA.inv(J),M - np.cross(Omega, np.matmul(J, Omega)))
    # print(Omega_dot)

    ## extension work to be done 
    ##1. larger uncertainty values to test L1 performance
    ##2. plot the sigma hat vs. sigma injected
    ##3. unmatched uncertainty? uncertain values on the acceleration dynamics



    diff_state = np.array([vx,
              vy,
              vz,
              -thrust_proj[0],
              -thrust_proj[1],
              g - thrust_proj[2],
              Omega_dot[0],
              Omega_dot[1],
              Omega_dot[2]])
    return diff_state, uncertain_inj


def f_value(J, m, state,u):
    #Dynamics
    # state: [p(3), v(3), R_Matrix(9), omega(3)]
    ## p(3)-- x,y,z are position of drone in world/inertial frame
    ## v(3)-- vx,vy,vz are positional/linear velocities in world/inertial frame

    ## omega(3)-- rotational angular velocity in the body frame (from IMU)

    # control: [f, moments(3)]
    ## moments(3)-- the moments in the body frame
    g = 9.81
    vx = state[3]
    vy = state[4]
    vz = state[5]
    Rvec = state[6:15]
    Omega = state[15:18]  # angular velocity vector in body frame

    f_thrust = u[0]
    M = u[1:4]

    r_mat = Rvec.reshape(3,3)
    e3 = np.array([0,0,1])

    thrust_proj = f_thrust/m*np.matmul(r_mat,e3)

    Rdot = np.matmul(r_mat,hatOperator(Omega))
    # # angular velocity in the inertial frame
    # Thetadot = omega2thetadot(Omega, Theta)
    # angular acceleration in body frame
    Omega_dot = np.matmul(LA.inv(J),M - np.cross(Omega, np.matmul(J, Omega)))
    # print(Omega_dot)



    diff_state = np.array([vx,
              vy,
              vz,
              -thrust_proj[0],
              -thrust_proj[1],
              g - thrust_proj[2],
              Omega_dot[0],
              Omega_dot[1],
              Omega_dot[2]])
    return diff_state



# def thetadot2omega(thetadot, Theta):
#     # convert the angular velocity from the world/inertial frame to the body frame
#     phi = Theta[0]
#     theta = Theta[1]
#     psi = Theta[2]

#     trans_mat = np.array([[1,0,-np.sin(theta)],
#                           [0,np.cos(phi),np.cos(theta)*np.sin(phi)],
#                           [0,-np.sin(phi),np.cos(theta)*np.cos(phi)]])
#     omega = np.matmul(trans_mat,thetadot)
#     return omega

# def omega2thetadot(Omega, Theta):
#     # convert the angular velocity from the body frame to the world/inertial frame

#     phi = Theta[0]
#     theta = Theta[1]
#     psi = Theta[2]

#     trans_mat = np.linalg.inv(np.array([[1,0,-np.sin(theta)],
#                           [0,np.cos(phi),np.cos(theta)*np.sin(phi)],
#                           [0,-np.sin(phi),np.cos(theta)*np.cos(phi)]]))
#     Thetadot = np.matmul(trans_mat,Omega)

#     return Thetadot

# def sim_control():
#     thrustMomentCmd = np.array([0.0,0.0,0.0,0.0])
#     # thrustMomentCmd[0] = 0.752*9.81+0.02
#     thrustMomentCmd[3] = 1.0
#     return thrustMomentCmd


def geo_control(J, kg_vehicleMass, currentTime,state):

    # GeoCtrl_Kpx = 4.5 # 4.512
    # GeoCtrl_Kpy = 5.0 #4.512

    # GeoCtrl_Kpz = 5
    # GeoCtrl_Kvx = 0.5
    # GeoCtrl_Kvy = 0.6 # 0.5
    # GeoCtrl_Kvz = 1.504
    # GeoCtrl_KRx = 0.128
    # GeoCtrl_KRy = 0.086
    # GeoCtrl_KRz = 0.02
    # GeoCtrl_KOx = 0.07327586207

    # GeoCtrl_KOy = 0.05 # 0.073
    # GeoCtrl_KOz = 0.004
    # GeoCtrl_Kpx = 0.62 # 4.512
    # GeoCtrl_Kpy = 0.45 #4.512

    # GeoCtrl_Kpz = 0.69
    # GeoCtrl_Kvx = 0.1
    # GeoCtrl_Kvy = 0.1 # 0.5
    # GeoCtrl_Kvz = 0.15
    # GeoCtrl_KRx = 0.015
    # GeoCtrl_KRy = 0.015
    # GeoCtrl_KRz = 0.002
    # GeoCtrl_KOx = 0.0022

    # GeoCtrl_KOy = 0.0022 # 0.073
    # GeoCtrl_KOz = 0.0007
    # GeoCtrl_Kpx = 4.5 # 4.512 
    # GeoCtrl_Kpy = 5.0 #4.512   
    # GeoCtrl_Kpz = 5 
    # GeoCtrl_Kvx = 0.5 
    # GeoCtrl_Kvy = 0.6 # 0.5
    # GeoCtrl_Kvz = 1.504

    # GeoCtrl_KRx = 0.128 #0.128
    # GeoCtrl_KRy = 0.086
    # GeoCtrl_KRz = 0.02#0.02
    # GeoCtrl_KOx = 0.0732 #0.07327586207
    # GeoCtrl_KOy = 0.05 # 0.073
    # GeoCtrl_KOz = 0.004 #0.004
    GeoCtrl_Kpx = 16.*kg_vehicleMass # 4.512
    GeoCtrl_Kpy = 16.*kg_vehicleMass #4.512

    GeoCtrl_Kpz = 16.*kg_vehicleMass
    GeoCtrl_Kvx = 5.6*kg_vehicleMass
    GeoCtrl_Kvy = 5.6*kg_vehicleMass # 0.5
    GeoCtrl_Kvz = 5.6*kg_vehicleMass
    GeoCtrl_KRx = 8.81
    GeoCtrl_KRy = 8.81
    GeoCtrl_KRz = 8.81
    GeoCtrl_KOx = 2.54

    GeoCtrl_KOy = 2.54 # 0.073
    GeoCtrl_KOz = 2.54
    GRAVITY_MAGNITUDE = 9.81


    # phi = state[6]
    # theta = state[7]
    # psi = state[8]

    zeros2 = [0.0,0.0]
    zeros3 = [0.0,0.0,0.0]

    # targetPos.x = radius * sinf(currentRate * netTime)
    # targetPos.y = radius * (1 - cosf(currentRate * netTime))
    # targetPos.z = 1
    targetPos = np.array([2*(1-math.cos(currentTime)), 2*math.sin(currentTime), 1.0 + math.sin(currentTime)])

    # targetVel.x = radius * currentRate * cosf(currentRate * netTime)
    # targetVel.y = radius * currentRate * sinf(currentRate * netTime)
    # targetVel.z = 0
    targetVel = np.array([2*math.sin(currentTime), 2*math.cos(currentTime), math.cos(currentTime)])

    # targetAcc.x = -radius * currentRate * currentRate * sinf(currentRate * netTime)
    # targetAcc.y = radius * currentRate * currentRate * cosf(currentRate * netTime)
    # targetAcc.z = 0
    targetAcc = np.array([2*math.cos(currentTime), -2*math.sin(currentTime), -math.sin(currentTime)])


    # targetJerk.x = -radius * powF(currentRate,3) * cosf(currentRate * netTime)
    # targetJerk.y = -radius * powF(currentRate,3) * sinf(currentRate * netTime)
    # targetJerk.z = 0
    targetJerk = np.array([-2*math.sin(currentTime), -2*math.cos(currentTime), -math.cos(currentTime)])


    # targetSnap.x = radius * powF(currentRate,4) * sinf(currentRate * netTime)
    # targetSnap.y = -radius * powF(currentRate,4) * cosf(currentRate * netTime)
    # targetSnap.z = 0
    targetSnap = np.array([-2*math.cos(currentTime), 2*math.sin(currentTime), math.sin(currentTime)])


    targetYaw = np.array([1.0,0.0]) # represent the orientation vector (Algo 1 in the supplementary)
    targetYaw_dot = np.array(zeros2) # represent derivative of the orientation vector
    targetYaw_ddot = np.array(zeros2)
    # targetYaw = np.array([math.cos(currentTime), math.sin(currentTime)])
    # targetYaw_dot = np.array([-math.sin(currentTime), math.cos(currentTime)])
    # targetYaw_ddot = np.array([-math.cos(currentTime), -math.sin(currentTime)])

    # begin geometric control
    # Position Error (ep)
    statePos = np.array([state[0],state[1],state[2]])
    r_error = statePos - targetPos
    # print(r_error)

    # Velocity Error (ev)
    stateVel = np.array([state[3],state[4],state[5]])
    v_error = stateVel - targetVel
    # print(v_error)

    target_force = np.array(zeros3)
    target_force[0] = kg_vehicleMass * targetAcc[0] - GeoCtrl_Kpx * r_error[0] - GeoCtrl_Kvx * v_error[0]
    target_force[1] = kg_vehicleMass * targetAcc[1] - GeoCtrl_Kpy * r_error[1] - GeoCtrl_Kvy * v_error[1]
    target_force[2] = kg_vehicleMass * (targetAcc[2] - GRAVITY_MAGNITUDE) - GeoCtrl_Kpz * r_error[2] - GeoCtrl_Kvz * v_error[2]
    # target_force[2] = kg_vehicleMass * (targetAcc[2] + GRAVITY_MAGNITUDE) - GeoCtrl_Kpz * r_error[2] - GeoCtrl_Kvz * v_error[2]

    # r = Rot.from_euler('zyx', [phi, theta, psi], degrees=True) # ? I changed this, since no attitude (w,x,y,z) tuple is available
    # R = r.as_matrix()
    Rvec = state[6:15]
    R = Rvec.reshape(3,3)

    z_axis = R[:,2]

    # target thrust [F] (z-positive)
    target_thrust = -np.dot(target_force,z_axis)
    # target_thrust = np.dot(target_force,z_axis)
    # Calculate axis [zB_des] (z-positive)
    z_axis_desired = -target_force/np.linalg.norm(target_force)
    # z_axis_desired = target_force/np.linalg.norm(target_force)

    # [xC_des]
    # x_axis_desired = z_axis_desired x [cos(yaw), sin(yaw), 0]^T, b_int in the supplementary document
    x_c_des = np.array(zeros3)
    x_c_des[0] = targetYaw[0]
    x_c_des[1] = targetYaw[1]
    x_c_des[2] = 0

    x_c_des_dot = np.array(zeros3)
    x_c_des_dot[0] = targetYaw_dot[0]
    x_c_des_dot[1] = targetYaw_dot[1]
    x_c_des_dot[2] = 0

    x_c_des_ddot = np.array(zeros3)
    x_c_des_ddot[0] = targetYaw_ddot[0]
    x_c_des_ddot[1] = targetYaw_ddot[1]
    x_c_des_ddot[2] = 0
    # [yB_des]
    y_axis_desired = np.cross(z_axis_desired, x_c_des)
    y_axis_desired = y_axis_desired/LA.norm(y_axis_desired)

    # [xB_des]
    x_axis_desired = np.cross(y_axis_desired, z_axis_desired)

    # [eR]
    # Slow version
    Rdes = np.empty(shape=(3,3))
    Rdes[:,0] = x_axis_desired
    Rdes[:,1] = y_axis_desired
    Rdes[:,2] = z_axis_desired
    # Matrix3f Rdes(Vector3f(x_axis_desired.x, y_axis_desired.x, z_axis_desired.x),
    #               Vector3f(x_axis_desired.y, y_axis_desired.y, z_axis_desired.y),
    #               Vector3f(x_axis_desired.z, y_axis_desired.z, z_axis_desired.z));

    eRM = (np.matmul(Rdes.transpose(),R) - np.matmul(R.transpose(), Rdes)) / 2

    # Matrix3<T>(const T ax, const T ay, const T az,
    #  const T bx, const T by, const T bz,
    #  const T cx, const T cy, const T cz)
    # eR.x = eRM.c.y;
    # eR.y = eRM.a.z;
    # eR.z = eRM.b.x;
    eR = np.array(zeros3)
    eR = veeOperator(eRM)
    # print(eR)
    # eR[0] = eRM[2,1]
    # eR[1] = eRM[0,2]
    # eR[2] = eRM[1,0]

    Omega = np.array([state[15], state[16], state[17]])
    # print(Omega)

    #compute Omegad
    a_error = np.array(zeros3) # error on acceleration
    # a_error = [0,0,-GRAVITY_MAGNITUDE] + R[:,2]* target_thrust / kg_vehicleMass - targetAcc
    a_error = [0,0,GRAVITY_MAGNITUDE] - R[:,2]* target_thrust / kg_vehicleMass - targetAcc
    # ? turn GRAVITY_MAGNITUDE to - GRAVITY_MAGNITUDE
    # ? turn - R[:,2]* target_thrust / kg_vehicleMass to + R[:,2]* target_thrust / kg_vehicleMass

    target_force_dot = np.array(zeros3) # derivative of target_force
    target_force_dot[0] = - GeoCtrl_Kpx * v_error[0] - GeoCtrl_Kvx * a_error[0] + kg_vehicleMass * targetJerk[0]
    target_force_dot[1] = - GeoCtrl_Kpy * v_error[1] - GeoCtrl_Kvy * a_error[1] + kg_vehicleMass * targetJerk[1]
    target_force_dot[2] = - GeoCtrl_Kpz * v_error[2] - GeoCtrl_Kvz * a_error[2] + kg_vehicleMass * targetJerk[2]

    b3_dot = np.matmul(np.matmul(R, hatOperator(Omega)),np.array([0,0,1])) #derivative of (Re3) in eq (2)
    target_thrust_dot = - np.dot(target_force_dot,R[:,2]) - np.dot(target_force, b3_dot)
    # target_thrust_dot = + np.dot(target_force_dot,R[:,2]) + np.dot(target_force, b3_dot)
    # ? turn the RHS from - to +

    j_error = np.array(zeros3) # error on jerk
    # j_error = np.dot(R[:,2], target_thrust_dot) / kg_vehicleMass + b3_dot * target_thrust / kg_vehicleMass - targetJerk
    j_error = -np.dot(R[:,2], target_thrust_dot) / kg_vehicleMass - b3_dot * target_thrust / kg_vehicleMass - targetJerk
    # ? turn - np.dot(R[:,2], target_thrust_dot) / kg_vehicleMass to np.dot(R[:,2], target_thrust_dot) / kg_vehicleMass
    # ? turn - b3_dot * target_thrust / kg_vehicleMass to + b3_dot * target_thrust / kg_vehicleMass

    target_force_ddot = np.array(zeros3) # derivative of target_force_dot
    target_force_ddot[0] = - GeoCtrl_Kpx * a_error[0] - GeoCtrl_Kvx * j_error[0] + kg_vehicleMass * targetSnap[0]
    target_force_ddot[1] = - GeoCtrl_Kpy * a_error[1] - GeoCtrl_Kvy * j_error[1] + kg_vehicleMass * targetSnap[1]
    target_force_ddot[2] = - GeoCtrl_Kpz * a_error[2] - GeoCtrl_Kvz * j_error[2] + kg_vehicleMass * targetSnap[2]


    b3cCollection = np.array([zeros3,zeros3,zeros3]) # collection of three three-dimensional vectors b3c, b3c_dot, b3c_ddot
    b3cCollection = unit_vec(-target_force, target_force_dot, -target_force_ddot) # unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m
    
    b3c = np.array(zeros3)
    b3c_dot = np.array(zeros3)
    b3c_ddot = np.array(zeros3)

    b3c[0] = b3cCollection[0]
    b3c[1] = b3cCollection[1]
    b3c[2] = b3cCollection[2]

    b3c_dot[0] = b3cCollection[3]
    b3c_dot[1] = b3cCollection[4]
    b3c_dot[2] = b3cCollection[5]

    b3c_ddot[0] = b3cCollection[6]
    b3c_ddot[1] = b3cCollection[7]
    b3c_ddot[2] = b3cCollection[8]

    """some changes start here"""
    A2 = - np.matmul(hatOperator(x_c_des), b3c)
    A2_dot = - np.matmul(hatOperator(x_c_des_dot),b3c) - np.matmul(hatOperator(x_c_des), b3c_dot)
    A2_ddot = - np.matmul(hatOperator(x_c_des_ddot), b3c) - np.matmul(hatOperator(x_c_des_dot), b3c_dot) * 2 - np.matmul(hatOperator(x_c_des), b3c_ddot)

    b2cCollection = np.array([zeros3,zeros3,zeros3]) # collection of three three-dimensional vectors b2c, b2c_dot, b2c_ddot
    b2cCollection = unit_vec(A2, -A2_dot, A2_ddot) # unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    b2c = np.array(zeros3)
    b2c_dot = np.array(zeros3)
    b2c_ddot = np.array(zeros3)

    b2c[0] = b2cCollection[0]
    b2c[1] = b2cCollection[1]
    b2c[2] = b2cCollection[2]

    b2c_dot[0] = b2cCollection[3]
    b2c_dot[1] = b2cCollection[4]
    b2c_dot[2] = b2cCollection[5]

    b2c_ddot[0] = b2cCollection[6]
    b2c_ddot[1] = b2cCollection[7]
    b2c_ddot[2] = b2cCollection[8]

    b1c_dot = np.matmul(hatOperator(b2c_dot), b3c) + np.matmul(hatOperator(b2c), b3c_dot)
    b1c_ddot = np.matmul(hatOperator(b2c_ddot),b3c) + np.matmul(hatOperator(b2c_dot), b3c_dot) * 2 + np.matmul(hatOperator(b2c), b3c_ddot)

    Rd_dot = np.empty(shape=(3,3)) # derivative of Rdes
    Rd_ddot = np.empty(shape=(3,3)) # derivative of Rd_dot

    Rd_dot[0,:] = b1c_dot
    Rd_dot[1,:] = b2c_dot
    Rd_dot[2,:] = b3c_dot
    Rd_dot.transpose()

    Rd_ddot[0,:] = b1c_ddot
    Rd_ddot[1,:] = b2c_ddot
    Rd_ddot[2,:] = b3c_ddot
    Rd_ddot.transpose()

    Omegad = veeOperator(np.matmul(Rdes.transpose(), Rd_dot))
    # print(currentTime)
    # print(Omegad)
    Omegad_dot = veeOperator(np.matmul(Rdes.transpose(), Rd_ddot) - np.matmul(hatOperator(Omegad), hatOperator(Omegad)))

    # these two lines are remedy which is not supposed to exist in the code. There might be an error in the code above.
    # Omegad[1] = -Omegad[1]
    # Omegad_dot[1] = -Omegad_dot[1]
    # temporarily use zero Omegad
    ew = Omega -  np.matmul(np.matmul(R.transpose(), Rdes), Omegad)
    # Moment: simple version
    M = np.array(zeros3)
    M[0] = -GeoCtrl_KRx * eR[0] - GeoCtrl_KOx * ew[0]
    M[1] = -GeoCtrl_KRy * eR[1] - GeoCtrl_KOy * ew[1]
    M[2] = -GeoCtrl_KRz * eR[2] - GeoCtrl_KOz * ew[2]
    # Moment: full version
    M = M - np.matmul(J, (np.matmul(hatOperator(Omega), np.matmul(R.transpose(),np.matmul(Rdes, Omegad))) - np.matmul(R.transpose(), np.matmul(Rdes, Omegad_dot))))
    # ShengC: an additive term is the following
    momentAdd = np.cross(Omega, (np.matmul(J, Omega))) # J is the inertia matrix
    M = M +  momentAdd

    thrustMomentCmd = np.array([0.0,0.0,0.0,0.0])
    thrustMomentCmd[0] = target_thrust
    thrustMomentCmd[1] = M[0]
    thrustMomentCmd[2] = M[1]
    thrustMomentCmd[3] = M[2]

    # u = np.array([0.0,0.0,0.0,0.0])
    # motorAssignMatrix = np.array([[1, 1, 1, 1],
    #                               [-0.1, 0.1,-0.1, 0.1],
    #                               [-0.075, 0.075, 0.075, -0.075],
    #                               [-0.022, -0.022, 0.022, 0.022]])
    # u = np.matmul(LA.inv(motorAssignMatrix),thrustMomentCmd) # no need to re-assign to every motor in simulation
    u = thrustMomentCmd
    return u, targetPos, Rdes

def hatOperator(v):

    hat = np.zeros((3, 3), dtype=float)
    # breakpoint()
    hat[2,1] = v[0]
    hat[1,2] = -v[0]
    hat[0,2] = v[1]
    hat[2,0] = -v[1]
    hat[1,0] = v[2]
    hat[0,1] = -v[2]
    return hat

def veeOperator(input):

    output = np.zeros((3), dtype=float)
    # const T ax, const T ay, const T az,
    # const T bx, const T by, const T bz,
    # const T cx, const T cy, const T cz
    output[0] = input[2][1]
    output[1] = input[0][2]
    output[2] = input[1][0]

    return output

def unit_vec(q, q_dot, q_ddot):
    """unit vector function provides three different normalization method/scale to the three three-element vectors"""

    collection = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    nq = LA.norm(q)
    u = q / nq
    u_dot = q_dot / nq - q * np.dot(q, q_dot) / pow(nq,3)

    u_ddot = q_ddot / nq - q_dot / pow(nq,3) * (2 * np.dot(q, q_dot)) - q / pow(nq,3) * (np.dot(q_dot, q_dot) + np.dot(q, q_ddot)) + 3 * q / pow(nq,5) * pow(np.dot(q, q_dot),2)
    # breakpoint()
    collection[0 : 3] = u
    collection[3 : 6] = u_dot
    collection[6 : 9] = u_ddot

    # breakpoint()

    return collection


def initialize_results(res_array, num):
    """append empty array for each state element"""
    for i in range(num):
        res_array.append([])
    # for i in range(num):
    #     res_array[i].append(init_value[i])


def simple_uncertain_plot():
    """comparison between uncertainty estimated and the actual injected uncertainty"""
    fig =  plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
    axes = fig.add_subplot(2, 2, 1)
    # axes.plot(pos[0],pos[1],label='actual')
    # axes.plot(position_des[0],position_des[1],label='des')
    # axes.set_title('Horizontal Position Over Time')
    # axes.set_xlabel('time (s)')
    # axes.set_ylabel('horizontal (m)')

    axes.plot(time_index, sigma_m_hat[0], label= 'est')
    # axes.plot(time_index, sigma_m_inj[0],label='actual')
    axes.set_title('Thrust uncertainty Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('sigma_thrust (N)')
    axes.legend()

     # Angles over time
    axes = fig.add_subplot(2, 2, 2)
    axes.plot(time_index, sigma_m_hat[1], label= 'est')
    # axes.plot(time_index, sigma_m_inj[1],label='actual')
    axes.set_title('Roll moment uncertainty Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('sigma_roll (N.m)')
    axes.legend()

    # Vertical position plot
    axes = fig.add_subplot(2, 2, 3)
    axes.plot(time_index, sigma_m_hat[2], label= 'est')
    # axes.plot(time_index, sigma_m_inj[2],label='actual')
    axes.set_title('Pitch moment uncertainty Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('sigma_pitch (N.m)')
    axes.legend()


    axes = fig.add_subplot(2, 2, 4)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(time_index, sigma_m_hat[3], label= 'est')
    # axes.plot(time_index, sigma_m_inj[3],label='actual')
    axes.set_title('Yaw moment uncertainty Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('sigma_yaw (N.m)')
    axes.legend()

    plt.tight_layout(pad=0.4, w_pad=2.5, h_pad=2.0)
    plt.show()


def simple_debug_plot():
    """Plots the laterial position, vertical position, and Euler angles over time.
    This is a plot for comparison with the desired values (reference: https://github.com/brianwade1/quadcopter_with_PID_controller/blob/master/Quadcopter_main.py)
    """

    fig =  plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
    axes = fig.add_subplot(2, 3, 1)
    # axes.plot(pos[0],pos[1],label='actual')
    # axes.plot(position_des[0],position_des[1],label='des')
    # axes.set_title('Horizontal Position Over Time')
    # axes.set_xlabel('time (s)')
    # axes.set_ylabel('horizontal (m)')

    axes.plot(time_index, pos[0], label= 'actual')
    axes.plot(time_index, position_des[0],label='des')
    axes.set_title('Horizontal Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('horizontal (m)')
    axes.legend()

     # Angles over time
    axes = fig.add_subplot(2, 3, 2)
    axes.plot(time_index, pos[1], label= 'actual')
    axes.plot(time_index, position_des[1],label='des')
    axes.set_title('Lateral Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('lateral (m)')
    axes.legend()

    # Vertical position plot
    axes = fig.add_subplot(2, 3, 3)
    axes.plot(time_index, pos[2], label= 'actual')
    axes.plot(time_index, position_des[2], label= 'des')
    axes.set_title('Vertical Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('altitude (m)')

    axes.legend()

    axes = fig.add_subplot(2, 3, 4)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(time_index,euler_ang_des[0],label='des')
    # axes.plot(time_index,euler_ang[0],label='actual')
    axes.set_title('Phi(roll) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('phi (deg)')
    axes.legend()
    # axes.plot(time_index, position_des[0], label= 'x_des')
    axes = fig.add_subplot(2, 3, 5)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(time_index,euler_ang_des[1],label='des')
    # axes.plot(time_index,euler_ang[1],label='actual')
    axes.set_title('Theta(pitch) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('theta (deg)')
    axes.legend()

    axes = fig.add_subplot(2, 3, 6)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
  

    axes.plot(time_index,euler_ang_des[2],label='des')
    # axes.plot(time_index,euler_ang[2],label='actual')
    axes.set_title('Psi(yaw) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('psi (deg)')
    axes.legend()

    plt.tight_layout(pad=0.4, w_pad=2.5, h_pad=2.0)
    plt.show()


def simple_l1_plot():
    '''
    comparison between the translational and rotational velocity (prediction vs. real)
    '''
    fig =  plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
    axes = fig.add_subplot(2, 3, 1)
    axes.plot(time_index, vel_real[0], label= 'actual',linestyle = 'dotted')
    axes.plot(time_index, vel_pred[0],label='pred',linestyle = 'dashed')
    # axes.set_title('Horizontal Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('vx (m/s)')

     # Angles over time
    axes = fig.add_subplot(2, 3, 2)
    axes.plot(time_index, vel_real[1], label= 'actual',linestyle = 'dotted')
    axes.plot(time_index, vel_pred[1],label='pred',linestyle = 'dashed')
    # axes.set_title('Lateral Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('vy (m/s)')

    # Vertical position plot
    axes = fig.add_subplot(2, 3, 3)
    axes.plot(time_index, vel_real[2], label= 'actual',linestyle = 'dotted')
    axes.plot(time_index, vel_pred[2], label= 'pred',linestyle = 'dashed')
    # axes.set_title('Vertical Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('vz (m/s)')

    axes.legend()

    axes = fig.add_subplot(2, 3, 4)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(time_index,ang_vel_real[0],label='actual',linestyle = 'dotted')
    axes.plot(time_index,ang_vel_pred[0],label='pred',linestyle = 'dashed')
    # axes.set_title('Phi(roll) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('omega1 (rad/s)')
    axes.legend()
    # axes.plot(time_index, position_des[0], label= 'x_des')
    axes = fig.add_subplot(2, 3, 5)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(time_index,ang_vel_real[1],label='actual',linestyle = 'dotted')
    axes.plot(time_index,ang_vel_pred[1],label='pred',linestyle = 'dashed')
    # axes.set_title('Theta(pitch) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('omega2 (rad/s)')
    axes.legend()

    axes = fig.add_subplot(2, 3, 6)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
  

    axes.plot(time_index,ang_vel_real[2],label='actual',linestyle = 'dotted')
    axes.plot(time_index,ang_vel_pred[2],label='pred',linestyle = 'dashed')
    # axes.set_title('Psi(yaw) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('omega3 (rad/s)')
    axes.legend()

    plt.tight_layout(pad=0.4, w_pad=2.5, h_pad=2.0)
    plt.show()


def simple_ctrl_plot():
    '''
    comparison between baseline geometric control effort and with L1 augmentation case
    '''
    fig =  plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
    axes = fig.add_subplot(2, 2, 1)
    axes.plot(time_index, ctrl[0], label= 'baseline',linestyle = 'dotted')
    axes.plot(time_index, ctrll1[0],label='L1_aug',linestyle = 'dashed')
    # axes.set_title('Horizontal Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('thrust (N)')

     # Angles over time
    axes = fig.add_subplot(2, 2, 2)
    axes.plot(time_index, ctrl[1], label= 'baseline',linestyle = 'dotted')
    axes.plot(time_index, ctrll1[1],label='L1_aug',linestyle = 'dashed')
    # axes.set_title('Lateral Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('M1 (N.m)')

    # Vertical position plot
    axes = fig.add_subplot(2, 2, 3)
    axes.plot(time_index, ctrl[2], label= 'baseline',linestyle = 'dotted')
    axes.plot(time_index, ctrll1[2], label= 'L1_aug',linestyle = 'dashed')
    # axes.set_title('Vertical Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('M2 (N.m)')

    axes.legend()

    axes = fig.add_subplot(2, 2, 4)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(time_index,ctrl[3],label='baseline',linestyle = 'dotted')
    axes.plot(time_index,ctrll1[3],label='L1_aug',linestyle = 'dashed')
    # axes.set_title('Phi(roll) angle Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('M3 (N.m)')
    axes.legend()

    plt.tight_layout(pad=0.4, w_pad=2.5, h_pad=2.0)
    plt.show()

def L1AC(state, ctrlcmd, din, L1_params):
    (As_v, As_omega, dt, ctoffq1Thrust, ctoffq1Moment, ctoffq2Moment, kg_vehicleMass, GRAVITY_MAGNITUDE, J ) = L1_params
    (v_hat_prev, omega_hat_prev, R_prev, v_prev, omega_prev, 
    u_b_prev, u_ad_prev, sigma_m_hat_prev, sigma_um_hat_prev, 
    lpf1_prev, lpf2_prev) = din
    thrustMomentCmd = ctrlcmd
    # == begin L1 adaptive control ==
    # first do the state predictor
    e3 = np.array([0.0, 0.0, 1.0])
    # load translational velocity
    v_now = state[3:6]

    # load rotational velocity
    omega_now = state[15:18]

    massInverse = 1.0 / kg_vehicleMass

    # compute prediction error (on previous step)
    vpred_error_prev = v_hat_prev - v_prev # computes v_tilde for (k-1) step
    omegapred_error_prev = omega_hat_prev - omega_prev # computes omega_tilde for (k-1) step

    v_hat = v_hat_prev + (e3 * GRAVITY_MAGNITUDE - R_prev[:,2]* (u_b_prev[0] + u_ad_prev[0] + sigma_m_hat_prev[0]) * massInverse + R_prev[:,0] * sigma_um_hat_prev[0] * massInverse + R_prev[:,1] * sigma_um_hat_prev[1] * massInverse + vpred_error_prev * As_v) * dt
    Jinv = LA.inv(J)
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

    # use the rotation matrix in the current step
    R = state[6:15].reshape(3,3)

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
    u_b_prev = thrustMomentCmd

    controlcmd_L1 = thrustMomentCmd + u_ad_prev

    din = (v_hat_prev, omega_hat_prev, R_prev, v_prev, omega_prev, 
    u_b_prev, u_ad_prev, sigma_m_hat_prev, sigma_um_hat_prev, 
    lpf1_prev, lpf2_prev)
    return controlcmd_L1, din

def L1_init(state):
    v_hat_prev = state[3:6]
    omega_hat_prev = state[15:18]
    R_prev = state[6:15].reshape(3,3)
    v_prev = np.array([0.0,0.0,0.0])
    omega_prev = np.array([0.0,0.0,0.0])

    u_b_prev = np.array([0.0,0.0,0.0,0.0])
    u_ad_prev = np.array([0.0,0.0,0.0,0.0])
    sigma_m_hat_prev = np.array([0.0,0.0,0.0,0.0])
    sigma_um_hat_prev = np.array([0.0,0.0])
    lpf1_prev = np.array([0.0,0.0,0.0,0.0])
    lpf2_prev = np.array([0.0,0.0,0.0,0.0])

    din = (v_hat_prev, omega_hat_prev, R_prev, v_prev, omega_prev, 
    u_b_prev, u_ad_prev, sigma_m_hat_prev, sigma_um_hat_prev, 
    lpf1_prev, lpf2_prev)
    return din


def simulate(s, e, dt, init):
    # Simulation times, in seconds
    start_t = s
    end_t = e

    # system parameters
    J = 1e-3*np.diag([2.5, 2.1, 4.3]) #same as github code
    # J = np.diag([0.0820, 0.0845, 0.1377]) # same as gwu code
    # J = 1e-5*np.diag([5.8, 7.2, 10])
    m = 0.752 #same as github code
    # m = 4.34 #same as gwu code
    # m = 0.075
    g = 9.81

    # Number of points in the simulation
    N_step = int((end_t - start_t)/dt)

    # Initial simulation states
    x = np.array(init[0:3])
    # x = np.array([0,0,1])
    # xdot = np.array([0,2,1])
    # x = np.zeros((3,))
    xdot = np.array(init[3:6])


    # Rvec = np.zeros((9,))
    # r = Rot.from_euler('zyx', [0, 0, 0], degrees=True)
    # Rinit = r.as_matrix()
    # Rinit = np.identity(3)
    Rvec = np.array(init[6:15])

    # simulate some disturbance in the angular velocity
    # deviation = 100
    # Theta = np.array([0,0,0])
    # Thetadot = np.radians(2 * deviation * np.random.rand(3,) - deviation)
    # Thetadot = np.array([0,0,0])
    # Omega = thetadot2omega(Thetadot,Theta)
    # Omega = np.array([0.18760213,0.10139737,0])
    Omega = np.array(init[15:18])

    state = np.concatenate((x, xdot, Rvec, Omega))
    # print(state.shape)


    # Initialize result arrays
    position = []
    initialize_results(position,3)

    position_des = []
    initialize_results(position_des,3)

    velocity = []
    initialize_results(velocity, 3)

    # angle = []
    # initialize_results(angle, 3)

    angle_vel = []
    initialize_results(angle_vel,3)

    # euler_ang = []
    # initialize_results(euler_ang,3)

    euler_ang_des = []
    initialize_results(euler_ang_des,3)

    control = []
    initialize_results(control,4)

    vel_real = []
    initialize_results(vel_real,3)

    vel_pred = []
    initialize_results(vel_pred,3)

    ang_vel_real = []
    initialize_results(ang_vel_real,3)

    ang_vel_pred = []
    initialize_results(ang_vel_pred,3)

    controll1 = []
    initialize_results(controll1,4)

    sigma_m_hat = []
    initialize_results(sigma_m_hat,4)

    sigma_m_inj = []
    initialize_results(sigma_m_inj,4)

    ##L1-related parameters
    As_v = -5.0 # parameter for L1
    As_omega = -5.0 # parameter for L1
    dt_L1 = 0.001 # sample time for L1 AC, for simplicity, set the same as the simulation step size
    """ For large uncertainties ..."""
    ctoffq1Thrust = 5*7 # cutoff frequency for thrust channel LPF (rad/s)
    ctoffq1Moment = 1*7 # cutoff frequency for moment channels LPF1 (rad/s)
    ctoffq2Moment = 1*7 # cutoff frequency for moment channels LPF2 (rad/s)
    # """ Original LPF Ctoff freq settings ..."""
    # ctoffq1Thrust = 5 # cutoff frequency for thrust channel LPF (rad/s)
    # ctoffq1Moment = 1 # cutoff frequency for moment channels LPF1 (rad/s)
    # ctoffq2Moment = 1 # cutoff frequency for moment channels LPF2 (rad/s)

    L1_params = (As_v, As_omega, dt_L1, ctoffq1Thrust, ctoffq1Moment, ctoffq2Moment, m, g, J )

    din = L1_init(state) # initialization of L1 inputs
    times = np.linspace(start_t, end_t, num=N_step)
    res = []
    for t in times:
        res.append(np.concatenate(([t], state)))
        x = np.array([state[0], state[1], state[2]])
        xdot = np.array([state[3], state[4], state[5]])
        Rvec = state[6:15]
        Rmat = Rvec.reshape(3,3)
        # print(Rmat)
        # rnow = Rot.from_matrix(Rmat)
        # euler_now = rnow.as_euler('zyx', degrees=True)
        # print(euler_now)


        Omega = np.array([state[15], state[16], state[17]])

        controlcmd, pos_des, Rdes = geo_control(J, m, t, state)
        controlcmd_L1, din = L1AC(state, controlcmd, din, L1_params)
        # print(controlcmd)
        # print('------------------------------------------')
        # print(controlcmd_L1)
        # export data for comparison
        vel_now = din[3] 
        vel_pred_now = din[0]
        ang_vel_now = din[4]
        ang_vel_pred_now = din[1]
        sigma_m_hat_now = din[7]
        # print(vel_now)
        # print(vel_pred_now)


        # controlcmd = sim_control()
        rdes = Rot.from_matrix(Rdes)
        euler_des = rdes.as_euler('zyx', degrees=True)
        # euler_err= np.array([euler_now[2]-euler_des[2], euler_now[1]-euler_des[1], euler_now[0]-euler_des[0]])
        # print(euler_err)
        position[0].append(x[0])
        position[1].append(x[1])
        position[2].append(x[2])

        position_des[0].append(pos_des[0])
        position_des[1].append(pos_des[1])
        position_des[2].append(pos_des[2])

        velocity[0].append(xdot[0])
        velocity[1].append(xdot[1])
        velocity[2].append(xdot[2])

        # euler_ang[0].append(euler_now[2]) #roll (x) angle
        # euler_ang[1].append(euler_now[1]) #pitch (y) angle
        # euler_ang[2].append(euler_now[0]) #yaw (z) angle

        euler_ang_des[0].append(euler_des[2])
        euler_ang_des[1].append(euler_des[1])
        euler_ang_des[2].append(euler_des[0])

        angle_vel[0].append(Omega[0])
        angle_vel[1].append(Omega[1])
        angle_vel[2].append(Omega[2])

        control[0].append(controlcmd[0])
        control[1].append(controlcmd[1])
        control[2].append(controlcmd[2])
        control[3].append(controlcmd[3])

        controll1[0].append(controlcmd_L1[0])
        controll1[1].append(controlcmd_L1[1])
        controll1[2].append(controlcmd_L1[2])
        controll1[3].append(controlcmd_L1[3])

        vel_real[0].append(vel_now[0])
        vel_real[1].append(vel_now[1])
        vel_real[2].append(vel_now[2])

        vel_pred[0].append(vel_pred_now[0])
        vel_pred[1].append(vel_pred_now[1])
        vel_pred[2].append(vel_pred_now[2])

        ang_vel_real[0].append(ang_vel_now[0])
        ang_vel_real[1].append(ang_vel_now[1])
        ang_vel_real[2].append(ang_vel_now[2])

        ang_vel_pred[0].append(ang_vel_pred_now[0])
        ang_vel_pred[1].append(ang_vel_pred_now[1])
        ang_vel_pred[2].append(ang_vel_pred_now[2])

        sigma_m_hat[0].append(sigma_m_hat_now[0])
        sigma_m_hat[1].append(sigma_m_hat_now[1])
        sigma_m_hat[2].append(sigma_m_hat_now[2])
        sigma_m_hat[3].append(sigma_m_hat_now[3])



        # diff_state = f_value(J, m, state, controlcmd_L1)
        diff_state = f_value(J, m, state, controlcmd_L1)

        # sigma_m_inj[0].append(uncertainty_injected[0])
        # sigma_m_inj[1].append(uncertainty_injected[1])
        # sigma_m_inj[2].append(uncertainty_injected[2])
        # sigma_m_inj[3].append(uncertainty_injected[3])


        Omegadot = diff_state[6:9]
        acc = diff_state[3:6]
        vel = diff_state[0:3]

        # update states based on time step
        Rmat = np.matmul(Rmat,expm(hatOperator(Omega)*dt))
        Omega = Omega + dt*Omegadot
        # print(Omega)


        # Theta = Theta + dt*np.array([diff_state[6],diff_state[7],diff_state[8]])
        xdot = xdot + dt*acc
        # x = x + dt*xdot
        x = x + dt*vel
        Rvec = Rmat.reshape(-1)
        state = np.concatenate((x, xdot, Rvec, Omega))

    return np.array(res), position, velocity, angle_vel, control, position_des, euler_ang_des, vel_real, vel_pred, ang_vel_real, ang_vel_pred, controll1, sigma_m_hat, sigma_m_inj

if __name__ == "__main__":
    sim_start = 0.0
    sim_end = 10.0
    dt = 0.001

    pos, vel, ang_vel, ctrl, position_des, euler_ang_des, vel_real, vel_pred, ang_vel_real, ang_vel_pred, ctrll1, sigma_m_hat, sigma_m_inj = simulate(sim_start, sim_end, dt)
    time_index = np.arange(sim_start, sim_end, dt)
    plt.plot(pos[0], pos[1])
    plt.show()
    # simple_debug_plot() #geometric control specific
    # simple_l1_plot()
    # simple_uncertain_plot()

