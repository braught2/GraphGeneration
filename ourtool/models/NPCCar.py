import numpy as np
from scipy.integrate import ode
from typing import List 

from ourtool.map.Map import Map

def dynamic(t, state, u):
    x, y, theta, v = state
    delta, a = u  
    x_dot = v*np.cos(theta+delta)
    y_dot = v*np.sin(theta+delta)
    theta_dot = v/1.75*np.sin(delta)
    v_dot = a 
    return [x_dot, y_dot, theta_dot, v_dot]

class NPCCar:
    def __init__(self, id:int = None):
        self.x:float = None
        self.y:float = None 
        self.theta:float = None 
        self.v:float = None 

        self.vehicle_mode: str = None 
        self.vehicle_lane: int = None 

        self.state_variable_continuous = ['x','y','theta','v']
        self.state_variable_discrete = ['vehicle_model','vehicle_lane']

        self.id = id

def TC_Simulate(Mode: str, initialCondition: List[float], time_bound: float):
    mode = Mode.split(',')
    vehicle_mode = mode[0]
    vehicle_lane = mode[1]
    time_step = 0.01
    time_bound = float(time_bound)
    number_points = int(np.ceil(time_bound/time_step))
    t = [i*time_step for i in range(0, number_points)]

    init = initialCondition
    trace = [[0]+init]
    if vehicle_mode == "Normal":
        for i in range(len(t)):
            x, y, theta, v = init 
            d = -y 
            psi = -theta
            steering = psi + np.arctan2(0.45*d, v)
            steering = np.clip(steering, -0.61, 0.61)
            a = 0
            r = ode(dynamic)    
            r.set_initial_value(init).set_f_params([steering, a])      
            res = r.integrate(r.t + time_step)
            init = res.flatten.tolist()
            trace.append([t[i] + time_step] + init)  
    elif vehicle_mode == "Brake":
        for i in range(len(t)):
            x, y, theta, v = init 
            d = -y 
            psi = -theta
            steering = psi + np.arctan2(0.45*d, v)
            steering = np.clip(steering, -0.61, 0.61)
            a = -1
            r = ode(dynamic)    
            r.set_initial_value(init).set_f_params([steering, a])      
            res = r.integrate(r.t + time_step)
            init = res.flatten.tolist()
            trace.append([t[i] + time_step] + init)  
    elif vehicle_mode == "Accel":
        for i in range(len(t)):
            x, y, theta, v = init 
            d = -y 
            psi = -theta
            steering = psi + np.arctan2(0.45*d, v)
            steering = np.clip(steering, -0.61, 0.61)
            a = 1
            r = ode(dynamic)    
            r.set_initial_value(init).set_f_params([steering, a])      
            res = r.integrate(r.t + time_step)
            init = res.flatten.tolist()
            trace.append([t[i] + time_step] + init)  
    elif vehicle_mode == "Stop":
        for i in range(len(t)):
            x, y, theta, v = init 
            d = -y 
            psi = -theta
            steering = psi + np.arctan2(0.45*d, v)
            steering = np.clip(steering, -0.61, 0.61)
            a = 0
            r = ode(dynamic)    
            r.set_initial_value(init).set_f_params([steering, a])      
            res = r.integrate(r.t + time_step)
            init = res.flatten.tolist()
            trace.append([t[i] + time_step] + init)  
    
    return np.array(trace) 
