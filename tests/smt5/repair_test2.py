"""
repair_test2.py
Description:
    Tests how quickly we can edit problems defined for the 
"""
from gurobipy import *
import sys, yaml, copy, time

import numpy as np

sys.path.append("../../")
from src.smt.simultaneous_synthesis import MAPlanRepairman_SMT
from PWLPlan import Node, IntFeasTol, add_space_constraints, add_time_constraints, \
                    add_velocity_constraints, add_mutual_clearance_constraints
from src.smt.constraints import add_time_constraints_smt, add_velocity_constraints_smt

from examples.repair.benchmark1_corner.corner_benchmark1 import create_optimistic_context

sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3

"""
test1
Description:
    Now, I would like to use SMT to solve the binary part of the problem ONLY.
"""
def test1():
    # Constants
    size_list = []
    tmax = 10.0
    repairman0 = MAPlanRepairman_SMT()

    # Algorithm Inputs

    x0s = [
        np.array([0.0,0.5]),
        np.array([0.5,-0.5]),
        np.array([-0.5,-0.5])
    ]

    num_agents = len(x0s)

    agent_radius = 0.22
    tmax=10.0
    vmax=3.0

    wall_half_width = 0.05
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    walls = []

    walls.append(np.array([-5, -5, -4, 4], dtype = np.float64))
    walls.append(np.array([10, 10, -4, 4], dtype = np.float64))
    walls.append(np.array([-5, 10, -4, -4], dtype = np.float64))
    walls.append(np.array([-5, 10, 4, 4], dtype = np.float64))

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
    goals = [(A, b1), (A, b2), (A, b3) ]

    specs = []
    for i in range(num_agents):
        avoids = [Node('negmu', info={'A':A, 'b':b}) for A, b in obs]
        # avoids += [Node('negmu', info={'A':goals[j][0], 'b':goals[j][1]}) for j in range(4) if j is not i]
        avoid_obs = Node('and', deps=avoids)
        always_avoid_obs = Node('A', deps=[avoid_obs,], info={'int':[0,tmax]})
        reach_goal = Node('mu', info={'A':goals[i][0], 'b':goals[i][1]})
        finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0, tmax]})
        specs.append(Node('and', deps=[always_avoid_obs, finally_reach_goal]))

    bloat = 0.1

    num_segs=6
    MIPGap = 0.3

    repairman0.add_binary_assignment_constraints(
        x0s, specs, bloat, num_segs,
        tmax=tmax,MIPGap=MIPGap
    )

    start = time.time()
    repairman0.smt_solver.check()
    end = time.time()
    print('solving it takes %.3f s'%(end - start))

    return end - start, print(repairman0.smt_solver.model())

"""
test2
Description:
    Now, I would like to see how easy it is to make the SMT solver NOT choose a certain combination of binary variables.
"""
def test2():
    # Constants
    size_list = []
    tmax = 10.0
    repairman0 = MAPlanRepairman_SMT()

    # Algorithm Inputs

    x0s = [
        np.array([0.0,0.5]),
        np.array([0.5,-0.5]),
        np.array([-0.5,-0.5])
    ]

    num_agents = len(x0s)

    agent_radius = 0.22
    tmax=10.0
    vmax=3.0

    wall_half_width = 0.05
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    walls = []

    walls.append(np.array([-5, -5, -4, 4], dtype = np.float64))
    walls.append(np.array([10, 10, -4, 4], dtype = np.float64))
    walls.append(np.array([-5, 10, -4, -4], dtype = np.float64))
    walls.append(np.array([-5, 10, 4, 4], dtype = np.float64))

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
    goals = [(A, b1), (A, b2), (A, b3) ]

    specs = []
    for i in range(num_agents):
        avoids = [Node('negmu', info={'A':A, 'b':b}) for A, b in obs]
        # avoids += [Node('negmu', info={'A':goals[j][0], 'b':goals[j][1]}) for j in range(4) if j is not i]
        avoid_obs = Node('and', deps=avoids)
        always_avoid_obs = Node('A', deps=[avoid_obs,], info={'int':[0,tmax]})
        reach_goal = Node('mu', info={'A':goals[i][0], 'b':goals[i][1]})
        finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0, tmax]})
        specs.append(Node('and', deps=[always_avoid_obs, finally_reach_goal]))

    bloat = 0.1

    num_segs=6
    MIPGap = 0.3

    repairman0.add_binary_assignment_constraints(
        x0s, specs, bloat, num_segs,
        tmax=tmax,MIPGap=MIPGap
    )

    start = time.time()
    repairman0.smt_solver.check()
    end = time.time()
    print('solving it takes %.3f s'%(end - start))

    return end - start, print(repairman0.smt_solver.model())

if __name__ == '__main__':
    # Constants

    # Algorithm
    model_info1 = test1()
    model_info2 = test2()

    test_data = {
        "test1 full replanning struct": model_info1,
        "test2 (function implementing checking code)": model_info2
    }

    # Save Results
    with open('smt-repair-tests2.yml','w') as outfile:
        yaml.dump(test_data, outfile, default_flow_style=False)