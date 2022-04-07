from multiprocessing.sharedctypes import Value
import warnings
import json
import copy

from ourtool.automaton.hybrid_io_automaton import HybridIoAutomaton
from ourtool.models.base_agent import BaseAgent
from ourtool.automaton.hybrid_automaton import HybridAutomaton
from ourtool.automaton.guard import Guard

class Scenario:
    def __init__(self, Map = None):
        self.map_automaton = None 
        if Map is not None:
            self.map_automaton = self.construct_map_automaton(Map)
        self.agent_list = []
        self.agent_automaton_list = []
        self.scenario_automaton:HybridIoAutomaton = None
        
    def add_agent(self, agent):
        self.agent_list.append(agent)
        
        # agent_automaton = self.construct_agent_automaton(agent)
        # self.agent_automaton_list.append(agent_automaton)
        
    def compose_agent_automaton(self):
        agent_automaton = HybridIoAutomaton(
            id = "ego",
            input_variables = ["other_x","other_y","other_theta","other_v"],
            output_variables = ['ego_x','ego_y','ego_theta','ego_v0'],
            discrete_variables = ['ego_vehicle_mode','ego_vehicle_lane'],
            modes = ['Normal,Lane0','Break,Lane0','Stop,Lane0'],
            edges = [[0,1],[0,2],[1,2],[1,0],[2,0]],
            guards = [
                Guard(logic_str="other_x-ego_x<20 and other_x-ego_x>10 and other_vehicle_lane!=ego_vehicle_lane"),
                Guard(logic_str="other_x-ego_x<10 and other_x-ego_x>-5"),
                Guard(logic_str="other_x-ego_x<10 and other_x-ego_x>-5"),
                Guard(logic_str="other_x-ego_x>30 or other_x-ego_x<-5"),
                Guard(logic_str="other_x-ego_x>30 or other_x-ego_x<-5")
            ],
            resets = ["","","","",""],
            dynamics = {"ourtool/models/InteractiveCar":['ego_x','ego_y','ego_theta','ego_v0']}
        )
        self.agent_automaton_list.append(agent_automaton)
        
        agent_automaton = HybridIoAutomaton(
            id = "other",
            output_variables = ["other_x","other_y","other_theta","other_v"],
            discrete_variables = ['other_vehicle_mode','other_vehicle_lane'],
            modes = ['Normal,Lane0'],
            dynamics = {"ourtool/models/NPCCar":["other_x","other_y","other_theta","other_v"]}
        )
        self.agent_automaton_list.append(agent_automaton)
        pass

    def add_map(self, map):
        self.map_automaton = self.construct_map_automaton(map)

    def construct_agent_automaton(agent:BaseAgent):
        agent_controller = agent.controller_fn
        # Generate automaton from file and combine it with the Map
        return HybridAutomaton()

    def construct_map_automaton(self, map):
        return HybridAutomaton()
    
    def construct_scenario_automaton(self):
        # Compose all automatons in the agent_automaton_list
        if len(self.agent_automaton_list) == 0:
            warnings.warn("No available agent automaton, cannot perform composition")
            return 
        
        scenario_automaton = copy.deepcopy(self.agent_automaton_list[0])
        for i in range(1, len(self.agent_automaton_list)):
            if not self.compatible(scenario_automaton, self.agent_automaton_list[i]):
                raise ValueError("Automatons not compatible")    
            scenario_automaton = self.compose(scenario_automaton, self.agent_automaton_list[i])

        self.scenario_automaton = scenario_automaton
        return scenario_automaton

    def compose(self, automaton1:HybridIoAutomaton, automaton2:HybridIoAutomaton) -> HybridIoAutomaton:
        # Get the variables for the composed automaton
        new_internal = automaton1.internal_variables + automaton2.internal_variables
        new_output = automaton1.output_variables + automaton2.output_variables
        new_input_union = set(automaton1.input_variables).union(automaton2.input_variables)-set(new_output)
        new_input = list(new_input_union)
        new_discrete = automaton1.discrete_variables + automaton2.discrete_variables

        # Get the modes for the composed automaton
        aut1_mode = automaton1.modes
        aut2_mode = automaton2.modes
        new_modes = []
        # Note that the new mode string in arranged in automaton1;automaton2
        if len(aut1_mode) == 0:
            new_modes = aut2_mode 
        elif len(aut2_mode) == 0:
            new_modes = aut1_mode
        else:
            for i in range(len(aut1_mode)):
                for j in range(len(aut2_mode)):
                    new_modes.append(aut1_mode[i] + ";" + aut2_mode[j])

        # Get the transitions in the composed automaton
        # Handle transitions in the first automaton
        # new_edge_list = []
        # new_guard_list = []
        # new_reset_list = []
        # for edge_idx, edge in enumerate(automaton1.edges):
        #     edge_src = edge[0]
        #     edge_dest = edge[1]
        #     edge_src_str = automaton1.modes[edge_src]
        #     edge_dest_str = automaton1.modes[edge_dest]
        #     for possible_mode in automaton2.modes:
        #         new_src_str = edge_src_str + ";" + possible_mode
        #         new_dest_str = edge_dest_str + ";" + possible_mode 
        #         new_src_idx = new_modes.index(new_src_str)
        #         new_dest_idx = new_modes.index(new_dest_str)
        #         new_edge_list.append([new_src_idx, new_dest_idx])
        #         new_guard_list.append(automaton1.guards[edge_idx].generate_guard_string())
        #         new_reset_list.append(automaton1.resets[edge_idx])

        # Handle transitions in the first automaton
        new_edge_list = []
        new_guard_list = []
        new_reset_list = []
        for edge_idx, edge in enumerate(automaton1.edges):
            edge_src = edge[0]
            edge_dest = edge[1]
            edge_src_str = automaton1.modes[edge_src]
            edge_dest_str = automaton1.modes[edge_dest]
            for possible_mode in automaton2.modes:
                # Get the two possible transition srcs for automaton1 and automaton2
                new_src_str = edge_src_str + ";" + possible_mode
                new_dest_str = edge_dest_str + ";" + possible_mode 
                
                # Plug the two pairs into the guard and see if the guard can be satisfied
                new_guard:Guard = copy.deepcopy(automaton1.guards[edge_idx])
                discrete_variable_dict = {}
                edge_src_split = edge_src_str.split(';')
                tmp = []
                for substr in edge_src_split:
                    tmp += substr.split(',')
                edge_src_split = tmp
                assert len(edge_src_split) == len(automaton1.discrete_variables)
                for i in range(len(automaton1.discrete_variables)):
                    discrete_variable_dict[automaton1.discrete_variables[i]] = '"'+edge_src_split[i]+'"'
                
                edge_src_split = possible_mode.split(';')
                tmp = []
                for substr in edge_src_split:
                    tmp += substr.split(',')
                edge_src_split = tmp
                assert len(edge_src_split) == len(automaton2.discrete_variables)
                for i in range(len(automaton2.discrete_variables)):
                    discrete_variable_dict[automaton2.discrete_variables[i]] = '"'+edge_src_split[i]+'"'

                execution_result = new_guard.execute_guard(automaton1, automaton2, discrete_variable_dict)
                # If yes, generate the transition
                if execution_result:
                    new_src_str = edge_src_str + ";" + possible_mode
                    new_dest_str = edge_dest_str + ";" + possible_mode 
                    new_src_idx = new_modes.index(new_src_str)
                    new_dest_idx = new_modes.index(new_dest_str)
                    new_edge_list.append([new_src_idx, new_dest_idx])
                    new_guard_list.append(new_guard.generate_guard_string())
                    new_reset_list.append(automaton1.resets[edge_idx])
                else:
                    continue

                # If no, continue
            

        # Handle transitions in the second automaton
        for edge_idx, edge in enumerate(automaton2.edges):
            edge_src = edge[0]
            edge_dest = edge[1]
            edge_src_str = automaton2.modes[edge_src]
            edge_dest_str = automaton2.modes[edge_dest]
            for possible_mode in automaton1.modes:
                new_src_str = possible_mode + ";" + edge_src_str
                new_dest_str = possible_mode + ";" + edge_dest_str
                new_src_idx = new_modes.index(new_src_str)
                new_dest_idx = new_modes.index(new_dest_str)
                new_edge_list.append([new_src_idx, new_dest_idx])
                new_guard_list.append(automaton2.guards[edge_idx].generate_guard_string())
                new_reset_list.append(automaton2.resets[edge_idx])


        # Get the dynamics in the composed automaton
        new_dynamics = automaton1.dynamics
        # aut2_dynamics = automaton2.dynamics 
        new_dynamics.update(automaton2.dynamics)

        # Generate the new hybrid automaton
        new_automaton = HybridIoAutomaton(
            input_variables = new_input,
            output_variables = new_output,
            internal_variables = new_internal,
            discrete_variables = new_discrete,
            modes = new_modes,
            edges = new_edge_list,
            guards = new_guard_list,
            resets = new_reset_list,
            dynamics = new_dynamics
        )
        return new_automaton

    def dump_scenario_automaton(self, fn):
        automaton_dict = self.scenario_automaton.generate_automaton_json()
        with open(fn,'w+') as f:
            json.dump(automaton_dict, f, indent=4)

    def compatible(self, automaton1, automaton2):
        return True