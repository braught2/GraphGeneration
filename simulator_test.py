import json
import ast 

import numpy as np
from scipy.integrate import ode
import matplotlib.pyplot as plt 

from ourtool.simulator.simulator import Simulator
from ourtool.automaton.guard import Guard
from pythonparser import parse_code

class Agent:
    def __init__(self, id):
        self.id = id

    @staticmethod
    def dynamic(t, state, u):
        x, y, theta, v = state
        delta, a = u  
        x_dot = v*np.cos(theta+delta)
        y_dot = v*np.sin(theta+delta)
        theta_dot = v/1.75*np.sin(delta)
        v_dot = a 
        return [x_dot, y_dot, theta_dot, v_dot]

    def TC_simulate(self, Mode, initialCondition, time_bound):
        mode = Mode.split(',')
        vehicle_mode = mode[0]
        vehicle_lane = mode[1]
        time_step = 0.01
        time_bound = float(time_bound)
        number_points = int(np.ceil(time_bound/time_step))
        t = [i*time_step for i in range(0,number_points)]

        init = initialCondition
        trace = [[0]+init]
        if vehicle_mode == "Normal":
            for i in range(len(t)):
                x,y,theta,v = init
                d = -y
                psi = -theta
                steering = psi + np.arctan2(0.45*d, v)
                steering = np.clip(steering, -0.61, 0.61)
                a = 0
                r = ode(self.dynamic)    
                r.set_initial_value(init).set_f_params([steering, a])      
                res:np.ndarray = r.integrate(r.t + time_step)
                init = res.flatten().tolist()
                trace.append([t[i] + time_step] + init) 
        elif vehicle_mode == "Switch_left":
            for i in range(len(t)):
                x,y,theta,v = init
                d = -y+1
                psi = -theta
                steering = psi + np.arctan2(0.45*d, v)
                steering = np.clip(steering, -0.61, 0.61)
                a = 0
                r = ode(Agent.dynamic)    
                r.set_initial_value(init).set_f_params([steering, a])      
                res:np.ndarray = r.integrate(r.t + time_step)
                init = res.flatten().tolist()
                trace.append([t[i] + time_step] + init) 
        elif vehicle_mode == "Switch_right":
            for i in range(len(t)):
                x,y,theta,v = init
                d = -y-1
                psi = -theta
                steering = psi + np.arctan2(0.45*d, v)
                steering = np.clip(steering, -0.61, 0.61)
                a = 0
                r = ode(self.dynamic)    
                r.set_initial_value(init).set_f_params([steering, a])      
                res:np.ndarray = r.integrate(r.t + time_step)
                init = res.flatten().tolist()
                trace.append([t[i] + time_step] + init) 
        return np.array(trace)

class transition_graph:
    def __init__(self, automaton):
        self.automaton = automaton 

    def get_all_transition(self, state_dict):
        # all_possible_transition = []
        # for agent_id in state_dict:
        #     agent_state, agent_mode = state_dict[agent_id]
        #     agent_mode_idx = self.automaton['vertex'].index(agent_mode)
        #     for edge_idx, edge in enumerate(self.automaton['edge']):
        #         guard_str = self.automaton['guards']
        #         guard = Guard(logic_str = guard_str)
        #         src_mode = agent_mode 
        #         dest_mode = self.automaton['vertex'][edge[1]] 


        agent1_state, agent1_mode = state_dict[0]
        x,y,theta,v = agent1_state[1:] 
        if x > 3 and x<5 and agent1_mode == "Normal,Lane0":
            return [
                (0, 'Normal,Lane0', 'Switch_left,Lane0', [x,y,theta,v]),
                (0, 'Normal,Lane0', 'Switch_right,Lane0', [x,y,theta,v]),
            ]
        if x > 10 and agent1_mode == "Switch_left,Lane0":
            return [(0, 'Switch_left,Lane0', 'Normal,Lane0', [x,y,theta,v])]
        if x > 10 and agent1_mode == "Switch_right,Lane0":
            return [(0, 'Switch_right,Lane0', 'Normal,Lane0', [x,y,theta,v])]
        return []

if __name__ == "__main__":
    agent1 = Agent(0)
    agent2 = Agent(1)
    tg = transition_graph(None)
    simulator = Simulator()

    # input_code_name = 'toythermomini.py' #sys.argv[1]
    # input_file_name = 'billiard_input.json' #sys.argv[2] 
    # output_file_name = 'out.json' #sys.argv[3]

    # with open(input_file_name) as in_json_file:
    #     input_json = json.load(in_json_file)


    # f = open(input_code_name,'r')
    # code = f.read()
    # tree = ast.parse(code)

    # output_dict = parse_code(code, input_json)


    root = simulator.simulate(
        [[0,0,0,0.5],[1,0,0,0.5]],
        ['Normal,Lane0','Normal,Lane0'],
        [agent1, agent2],
        tg,
        40
    )
    # print(res)
    # trace = agent1.TC_simulate("Normal,Lane0",[0,0,0,5],1)
    # print(trace)
    queue = [root]
    while queue!=[]:
        node = queue.pop(0)
        traces = node.trace
        agent_id = 0
        # for agent_id in traces:
        trace = np.array(traces[agent_id])
        plt.plot(trace[:,0], trace[:,2], 'b')
        # if node.child != []:
        queue += node.child 
    plt.show()