import sys
import numpy as np

sys.path.append("../../")
from PWLPlan import plan,plan2, Node
from src.plot_plan import plot_plan
from vis import vis

import matplotlib.pyplot as plt
import yaml

def test_clear3_clustering():
    x0s = [[2, 0.5], [7, 1.0] ]
    num_agents = len(x0s)

    wall_half_width = 0.05
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    walls = []

    walls.append(np.array([0, 0, 0, 4], dtype = np.float64))
    walls.append(np.array([10, 10, 0, 4], dtype = np.float64))
    walls.append(np.array([0, 10, 0, 0], dtype = np.float64))
    walls.append(np.array([0, 10, 4, 4], dtype = np.float64))

    walls.append(np.array([0, 4., 2, 2], dtype = np.float64))
    walls.append(np.array([6., 10, 2, 2], dtype = np.float64))

    obs = []
    for wall in walls:
        if wall[0]==wall[1]:
            wall[0] -= wall_half_width
            wall[1] += wall_half_width
        elif wall[2]==wall[3]:
            wall[2] -= wall_half_width
            wall[3] += wall_half_width
        else:
            raise ValueError('wrong shape for axis-aligned wall')
        wall *= np.array([-1,1,-1,1])
        obs.append((A, wall))

    b1 = np.array([-1.5, 2.5, -3, 4], dtype = np.float64)
    b2 = np.array([-7.5, 8.5, -3, 4], dtype = np.float64)
    b3 = np.array([-1.5, 2.5, 0, 1], dtype = np.float64)
    b4 = np.array([-7.5, 8.5, 0, 1], dtype = np.float64)
    goals = [(A, b1), (A, b2) ]

    tmax = 10.
    vmax = 3.

    specs = []
    for i in range(num_agents):
        avoids = [Node('negmu', info={'A':A, 'b':b}) for A, b in obs]
        # avoids += [Node('negmu', info={'A':goals[j][0], 'b':goals[j][1]}) for j in range(4) if j is not i]
        avoid_obs = Node('and', deps=avoids)
        always_avoid_obs = Node('A', deps=[avoid_obs,], info={'int':[0,tmax]})
        reach_goal = Node('mu', info={'A':goals[i][0], 'b':goals[i][1]})
        finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0, tmax]})
        specs.append(Node('and', deps=[always_avoid_obs, finally_reach_goal]))

    # print(x0s)
    sl = [ 0.11*4/2 , 0.8-2*(0.11*4/2)+(0.11*4/2) ]
    PWL, _ , _ , model_data = plan2(
        x0s, specs, bloat=0.2, num_segs=6, tmax=tmax, vmax=vmax, MIPGap = 0.3,
        size_list=sl
    )
    print(model_data)

    plots = [[goals, 'g'], [obs, 'k']]

    with open('3agents-clustered-free.yml','w') as outfile:
        yaml.dump(model_data, outfile, default_flow_style=True)

    #return x0s, plots, PWL
    fig1 = plot_plan(PWL,plots,
        size_list = sl
    )
    fig1.savefig("plot_plan1.png")

if __name__ == '__main__':
    test_clear3_clustering()