from ourtool.map import LaneMap, SingleStraightLaneMap
from ourtool.scenario import Scenario
from ourtool.models import InteractiveCar, NPCCar

modes:"ModeParam" = ["Normal", "Brake", "Stop"]

def controller(ego_state: InteractiveCar, npcs_state: NPCCar, map: LaneMap):
    if ego_state.x == "Normal":
        if npcs_state.x - ego_state.x < 20 and npcs_state.x - ego_state.x > 10:
            ego_state.vehicle_mode = "Brake"
        if npcs_state.x - ego_state.x < 10 and npcs_state.x - ego_state.x > -5:
            ego_state.vehicle_mode = "Stop"
    if ego_state.x == "Brake":
        if npcs_state.x - ego_state.x < 10 and npcs_state.x - ego_state.x > -5:
            ego_state.vehicle_mode = "Stop"
        if npcs_state.x - ego_state.x > 30 or npcs_state.x - ego_state.x < -5:
            ego_state.vehicle_mode = "Normal"
    if ego_state.x == "Stop":
        if npcs_state.x - ego_state.x > 30 or npcs_state.x - ego_state.x < -5:
            ego_state.vehicle_mode = "Normal"
    
if __name__ == "__main__":
    map = SingleStraightLaneMap()
    scenario = Scenario(Map = map)
    ego_car = InteractiveCar('./controller_simple2.py')
    npc_car = NPCCar()
    scenario.add_agent(ego_car)
    scenario.add_agent(npc_car)
    scenario.compose()
    # controller(ego_state, npcs_state, None)