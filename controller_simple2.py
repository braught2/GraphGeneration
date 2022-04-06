from typing import List

from ourtool.map.lane_map import LaneMap
from ourtool.map.single_straight_lane import SingleStraightLaneMap
from ourtool.scenario.Scenario import Scenario
from ourtool.models.base_agent import BaseAgent
from ourtool.models.InteractiveCar import InteractiveCar
from ourtool.models.NPCCar import NPCCar

modes:"ModeParam" = ["Normal", "Brake", "Stop"]

def controller(ego_state: InteractiveCar, other_state: NPCCar, map: LaneMap):
    if ego_state.x == "Normal":
        if other_state.x - ego_state.x < 20 and other_state.x - ego_state.x > 10:
            ego_state.vehicle_mode = "Brake"
        if other_state.x - ego_state.x < 10 and other_state.x - ego_state.x > -5:
            ego_state.vehicle_mode = "Stop"
    if ego_state.x == "Brake":
        if other_state.x - ego_state.x < 10 and other_state.x - ego_state.x > -5:
            ego_state.vehicle_mode = "Stop"
        if other_state.x - ego_state.x > 30 or other_state.x - ego_state.x < -5:
            ego_state.vehicle_mode = "Normal"
    if ego_state.x == "Stop":
        if other_state.x - ego_state.x > 30 or other_state.x - ego_state.x < -5:
            ego_state.vehicle_mode = "Normal"
    
if __name__ == "__main__":
    map = SingleStraightLaneMap()
    scenario:Scenario = Scenario(Map = map)
    ego_car = InteractiveCar('./controller_simple2.py')
    npc_car = NPCCar()
    scenario.add_agent(ego_car)
    scenario.add_agent(npc_car)
    scenario.compose_agent_automaton()
    scenario.construct_scenario_automaton()
    scenario.dump_scenario_automaton('tmp.json')
    # controller(ego_state, npcs_state, None)