class BaseAgent:
    def __init__(self):
        self.state_variable_continuous = []
        self.state_variable_discrete = []

        self.controller_fn = None 
        self.id = None

        self.dynamics = None

    def TC_Simulate(self, mode, initialSet, time_horizon):
        raise NotImplementedError