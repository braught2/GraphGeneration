from ourtool.automaton.guard import Guard

if __name__ == "__main__":
    tmp = Guard()
    tmp.construct_tree_from_str('other_x-ego_x<20 and other_x-ego_x>10 and other_vehicle_lane==ego_vehicle_lane')
    print("stop")