from ourtool.models import BaseAgent

class Scenario:
    def __init__(self, Map = None):
        self.map_automaton = None 
        if Map is not None:
            self.map_automaton = self.construct_map_automaton(Map)
        self.agent_automaton_list = []
        
    def add_agent(self, agent):
        agent_automaton = self.construct_agent_automaton(agent)
        self.agent_automaton_list.append(agent_automaton)
        
    def add_map(self, map):
        self.map_automaton = self.construct_map_automaton(map)

    def construct_agent_automaton(agent:BaseAgent):
        agent_controller = agent.controller_fn
        # Generate automaton from file and combine it with the Map
        pass 

    def construct_map_automaton(map):
        pass
    
    def compose(self):
        # Compose all automatons in the agent_automaton_list
        pass