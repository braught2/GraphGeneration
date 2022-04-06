from typing import List, Dict

class HybridAutomaton:
    def __init__(
        self,
        variables=[],
        modes=[],
        edges=[],
        guards=[],
        resets=[],
        dynamics={}
    ):
        self.variables = variables
        self.modes = modes
        self.edges = edges
        self.guards = guards
        self.resets = resets
        self.dynamics = dynamics

    
    def generate_automaton_json(self):
        automaton_dict = {}
        automaton_dict['variables'] = self.variables 
        automaton_dict['edge'] = self.edges 
        automaton_dict['guards'] = self.guards 
        automaton_dict['resets'] = self.resets
        automaton_dict['vertex'] = self.modes
        automaton_dict['directory'] = self.dynamics
        return automaton_dict 