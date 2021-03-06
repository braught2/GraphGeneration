from dryvr_plus_plus.scene_verifier.code_parser.parser import ControllerIR, Env


class BaseAgent:
    def __init__(self, id, code = None, file_name = None):  
        self.controller: ControllerIR = ControllerIR.parse(code, file_name)
        self.id = id

    def TC_simulate(self, mode, initialSet, time_horizon, time_step, map=None):
        raise NotImplementedError
