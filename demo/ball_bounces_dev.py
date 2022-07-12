from enum import Enum, auto
import copy
from typing import List

class BallTypeMode(Enum):
    TYPE1 = auto()
    TYPE2 = auto()

class BallMode(Enum):
    Normal = auto()
    
class State:
    x:float
    y = 0.0
    vx = 0.0
    vy = 0.0
    ball_mode: BallMode
    type: BallTypeMode
    def __init__(self, x, y, vx, vy, ball_mode:BallMode, type: BallTypeMode):
        pass

def controller(ego:State):
    output = copy.deepcopy(ego)
    if ego.x<0:
        output.vx = -ego.vx
        output.x=0
    if ego.y<0:
        output.vy = -ego.vy
        output.y=0
    if ego.x>20:
        output.vx = -ego.vx
        output.x=20
    if ego.y>20:
        output.vy = -ego.vy
        output.y=20
    return output

from dryvr_plus_plus.example.example_agent.ball_agent import BallAgent
from dryvr_plus_plus.scene_verifier.scenario.scenario import Scenario
from dryvr_plus_plus.example.example_map.simple_map2 import SimpleMap3
from dryvr_plus_plus.plotter.plotter2D import *
import plotly.graph_objects as go
from dryvr_plus_plus.example.example_sensor.fake_sensor import FakeSensor4
from dryvr_plus_plus.scene_verifier.sensor.base_sensor import BaseSensor

if __name__ == "__main__":
    ball_controller = './demo/ball_bounces_dev.py'
    bouncingBall = Scenario()
    myball1 = BallAgent('red-ball', file_name=ball_controller)
    bouncingBall.add_agent(myball1)
    bouncingBall.set_init(
        [
            [[5, 10, 2, 2], [5, 10, 2, 2]],
        ],
        [
            (BallMode.Normal,),
        ],
        [
            (BallTypeMode.TYPE1,),
        ]
    )
    # bouncingBall.set_sensor(FakeSensor4())
    traces = bouncingBall.simulate(10, 0.01)
    fig = go.Figure()
    fig = simulation_anime_trail(traces, fig=fig)
    fig.show()

