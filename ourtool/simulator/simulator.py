from typing import List, Dict
from ourtool.models.base_agent import BaseAgent
import numpy as np
import copy

class SimulationTreeNode:
    def __init__(
        self,
        trace={},
        init={},
        mode={},
        agent={},
        child=[],
        start_time = 0
    ):
        self.trace:Dict = trace
        self.init:Dict = init
        self.mode:Dict = mode
        self.agent:Dict = agent
        self.child: List[SimulationTreeNode] = child
        self.start_time:float = start_time

class Simulator:
    def __init__(self):
        self.simulation_tree_root = None

    def simulate(self, init_list, init_mode_list, agent_list:List[BaseAgent], transition_graph, time_horizon):
        # Setup the root of the simulation tree
        root = SimulationTreeNode()
        for i, agent in enumerate(agent_list):
            root.init[agent.id] = init_list[i]
            root.mode[agent.id] = init_mode_list[i]
            root.agent[agent.id] = agent
        self.simulation_tree_root = root
        simulation_queue = []
        simulation_queue.append(root)
        # Perform BFS through the simulation tree to loop through all possible transitions
        while simulation_queue != []:
            node:SimulationTreeNode = simulation_queue.pop(0)
            print(node.mode)
            remain_time = time_horizon - node.start_time
            if remain_time <= 0:
                continue
            # For trace not already simulated
            for agent_id in node.agent:
                if agent_id not in node.trace:
                    # Simulate the trace starting from initial condition
                    mode = node.mode[agent_id]
                    init = node.init[agent_id]
                    trace = node.agent[agent_id].TC_simulate(mode, init, remain_time)
                    trace[:,0] += node.start_time
                    node.trace[agent_id] = trace.tolist()

            trace_length = len(list(node.trace.values())[0])
            transitions = []
            for idx in range(trace_length):
                # For each trace, check with the guard to see if there's any possible transition
                # Store all possible transition in a list
                # A transition is defined by (agent, src_mode, dest_mode, corresponding reset, transit idx)
                # Here we enforce that only one agent transit at a time
                all_agent_state = {}
                for agent_id in node.agent:
                    all_agent_state[agent_id] = (node.trace[agent_id][idx], node.mode[agent_id])
                possible_transitions = transition_graph.get_all_transition(all_agent_state)
                if possible_transitions != []:
                    for agent_idx, src_mode, dest_mode, next_init in possible_transitions:
                        transitions.append((agent_idx, src_mode, dest_mode, next_init, idx))
                    break

            # truncate the computed trajectories from idx and store the content after truncate
            truncated_trace = {}
            for agent_idx in node.agent:
                truncated_trace[agent_idx] = node.trace[agent_idx][idx:]
                node.trace[agent_idx] = node.trace[agent_idx][:idx+1]

            # For each possible transition, construct the new node. 
            # Obtain the new initial condition for agent having transition
            # copy the traces that are not under transition
            for transition in transitions:
                transit_agent_idx, src_mode, dest_mode, next_init, idx = transition
                # next_node = SimulationTreeNode(trace = {},init={},mode={},agent={}, child = [], start_time = 0)
                next_node_mode = copy.deepcopy(node.mode) 
                next_node_mode[transit_agent_idx] = dest_mode 
                next_node_agent = node.agent 
                next_node_start_time = list(truncated_trace.values())[0][0][0]
                next_node_init = {}
                next_node_trace = {}
                for agent_idx in next_node_agent:
                    if agent_idx == transit_agent_idx:
                        next_node_init[agent_idx] = next_init 
                    else:
                        next_node_trace[agent_idx] = truncated_trace[agent_idx]
                
                # Put the node in the child of current node. Put the new node in the queue
                node.child.append(SimulationTreeNode(
                    trace = next_node_trace,
                    init = next_node_init,
                    mode = next_node_mode,
                    agent = next_node_agent,
                    child = [],
                    start_time = next_node_start_time
                ))
            simulation_queue += node.child
        return self.simulation_tree_root
