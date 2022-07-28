from quadrotor_agent import quadrotor_agent
from dryvr_plus_plus.scenario import Scenario
from dryvr_plus_plus.example.example_map.simple_map2 import SimpleMap2, SimpleMap3, SimpleMap5, SimpleMap6
from dryvr_plus_plus.plotter.plotter2D import *
from dryvr_plus_plus.example.example_sensor.fake_sensor import FakeSensor2
from plotter2D_old import plot_reachtube_tree
 
import plotly.graph_objects as go
from enum import Enum, auto
import os

import json
# from gen_json import write_json, read_json


class AgentMode(Enum):
    Default = auto()


if __name__ == "__main__":
    input_code_name = './quad_controller.py'
    scenario = Scenario()

    # step 1. create a quadrotor instance with the closed-loop dynamics
    quad = quadrotor_agent('quad1', file_name=input_code_name)
    scenario.add_agent(quad)
    # car = vanderpol_agent('car2', file_name=input_code_name)
    # scenario.add_agent(car)
    # scenario.set_sensor(FakeSensor2())
    # modify mode list input
    # Step 2. change the initial codnitions (set for all 18 states)
    scenario.set_init(
        [
        [     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.3],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.3]
        ] # tuning knobs (initial condition uncertainty)
            # [[1.25], [1.25]],
            # [[1.25, 2.25], [1.25, 2.25]],
            # [[1.55, 2.35], [1.55, 2.35]]
        ],
        [
            tuple([AgentMode.Default]),
            # tuple([AgentMode.Default]),
        ]
    )
    t_max = 10
    traces = scenario.verify(t_max, 0.05)
    traces.dump('output.json')
    # path = os.path.abspath(__file__)
    # path = path.replace('quadrotor_demo.py', 'output_geo_L1_TEST.json')
    # write_json(traces, path)


    N = 100*t_max + 1
    t = np.linspace(0,t_max,N)
    x_des_array = []
    y_des_array = []
    z_des_array = []

    for t_step in t:
        x_des_array.append(2*(1-np.cos(t_step)))
        y_des_array.append(2*np.sin(t_step))
        z_des_array.append(1.0 + np.sin(t_step))

    # fig = go.Figure()
    # """use these lines for generating x-y (phase) plots"""
    # fig = simulation_tree(traces, None, fig, 1, 2,
    #                       'lines', 'trace', print_dim_list=[1,2])
    # fig.add_trace(go.Scatter(x=x_des_array, y=y_des_array,mode="lines",line=dict(color="#0000ff"))) 
    # fig.update_yaxes(scaleanchor = "x",scaleratio = 1,)
    # fig.update_xaxes(range=[-0.5, 4.5],constrain="domain")
    # fig.show()

    # fig = go.Figure()
    import matplotlib.pyplot as plt 
    """use these lines for generating x-y (phase) plots"""
    fig = plot_reachtube_tree(traces.root, 'quad1', 1, [2])
    # fig.add_trace(go.Scatter(x=x_des_array, y=y_des_array,mode="lines",line=dict(color="#0000ff"))) 
    # fig.update_yaxes(scaleanchor = "x",scaleratio = 1,)
    # fig.update_xaxes(range=[-0.5, 4.5],constrain="domain")
    plt.show()

    """use these lines for generating x-t (time) plots"""
    # fig = reachtube_tree(traces, None, fig, 0, 1,
    #                       'lines', 'trace', print_dim_list=[0,1])
    # fig.add_trace(go.Scatter(x=t, y=x_des_array,mode="lines",line=dict(color="#0000ff"))) 
    # fig.update_xaxes(range=[-0.5, 10.5],constrain="domain")
    # fig.show()


    # fig_n = go.Figure()
    # fig_n = reachtube_tree(traces, None, fig_n, 0, 2,
    #                       'lines', 'trace', print_dim_list=[0,2])
    # fig_n.add_trace(go.Scatter(x=t, y=y_des_array,mode="lines",line=dict(color="#0000ff"))) 
    # fig_n.update_xaxes(range=[-0.5, 10.5],constrain="domain")
    # fig_n.show()

    # fig_new = go.Figure()
    # fig_new = reachtube_tree(traces, None, fig_new, 0, 3,
    #                       'lines', 'trace', print_dim_list=[0,3])
    # fig_new.add_trace(go.Scatter(x=t, y=z_des_array,mode="lines",line=dict(color="#0000ff"))) 
    # fig_new.update_xaxes(range=[-0.5, 10.5],constrain="domain")
    # fig_new.show()
