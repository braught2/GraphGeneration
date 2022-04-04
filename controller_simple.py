def controller(ego_state, npcs_state, map):
    x = ego_state.x 
    y = ego_state.y
    theta = ego_state.theta 
    vehicle_lane = ego_state.lane
    vehicle_mode = ego_state.vehicle_mode

    if vehicle_mode == "Straight":
        car_in_front = False 
        for npc in npcs_state:
            if condition:
                car_in_front = True
                break 
        if car_in_front:
            car_at_left = False 
            for npc in npcs_state:
                if condition2:
                    car_at_left = True
                    break 
            if not car_at_left:
                if map.have_left_lane(vehicle_lane):
                    vehicle_state = "Switch_left"
                else:
                    vehicle_state = "Stop"
            else:
                vehicle_state = "Stop"
    if vehicle_mode == "Switch_left":
        if condition(ego_state):
            vehicle_state = "Straight"
            vehicle_lane = map.left_lane(vehicle_lane)
    
    