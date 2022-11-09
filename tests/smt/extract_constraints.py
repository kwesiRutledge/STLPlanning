"""
extract_constraints.py
Description:
    This test script is meant to test the capability of extracting the constraints of a MILP for use in an SMT solver for fast solving of the binary variables.
"""

import sys
import numpy as np
from gurobipy import *

sys.path.append("../../")
from PWLPlan import Node, plan2
from src.plot_plan import plot_plan

sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3

"""
test1
Description:
    In this test, I would really like to extract the values out of a gurobi program and then attempt a warm start.
"""
def test1():
    x0s = [[2, 0.5], [6,0.5], [4,0.5]]
    num_agents = len(x0s)

    wall_half_width = 0.05
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    walls = []

    walls.append(np.array([0, 0, 0, 4], dtype = np.float64))
    walls.append(np.array([10, 10, 0, 4], dtype = np.float64))
    walls.append(np.array([0, 10, 0, 0], dtype = np.float64))
    walls.append(np.array([0, 10, 4, 4], dtype = np.float64))

    walls.append(np.array([0, 4., 2, 2], dtype = np.float64))
    walls.append(np.array([5., 10, 2, 2], dtype = np.float64))

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
    goals = [(A, b1), (A,b2) , (A,b1) ]

    tmax = 6.
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

    print(x0s)
    PWL, vars , var_vals , model_data = plan2(x0s, specs, bloat=0.2, num_segs=6, tmax=tmax, vmax=vmax, MIPGap = 0.3)
    fig = plot_plan(
        PWL, [[obs, 'k'],[goals,'m']]
    )
    fig.savefig("plan1.png")
    print(model_data)

    smt_bools = []
    for var_index in range(len(vars)):
        var = vars[var_index]
        var_type = model_data['TypeList'][var_index]
        var_value = var_vals[var_index]
        print("- ", var, var_type, var_value)

        # Define a set of z3 variables for each of the BINARY variables in vars
        if var_type == 'B':
            smt_bools.append(
                z3.Bool('b'+str(len(smt_bools)))
            )

if __name__ == '__main__':
    print("extract_constraints.py")
    test1()