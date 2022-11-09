"""
benchmark4-narrow-multi.py
Description:
    Creates code for the fourth benchmark problem.
    It involves four agents moving and trying to satisfy an 'until' specification.
"""

import numpy as np
import sys, yaml, time
sys.path.append('../../../')
from PWLPlan import plan2, Node
from src.plot_plan import plot_plan
from src.smt.simultaneous_synthesis import MAPlanRepairman_SMT

def define_optimistic_scenario():
    # Create Initial Conditions
    x0s = [[2, 0.5], [8, 0.5], [6,0.5], [4,0.5]]
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
    goals = [(A, b1), (A, b2), (A,b2) , (A,b1) ]

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

    bloat = 0.1
    PWL, _ , _ , model_data = plan2(x0s, specs, bloat=bloat, num_segs=6, tmax=tmax, vmax=vmax, MIPGap = 0.3,ignore_clearance_constraints=True)

    plots = [[goals, 'g'], [obs, 'k']]

    return PWL, plots, x0s, tmax, specs, bloat, model_data

"""
create_real_context
Description:
    Create context with a new obstacle in it.
"""
def create_real_context(plot_tuples):
    # Constants
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])

    # Extract obs from the plot tuples list
    plot_tuples1 = plot_tuples.copy()
    b = np.array([-1.9, 3.0, -2.25, 2.75], dtype = np.float64)
    plot_tuples1.append(
        [ [( np.vstack((np.eye(2),-np.eye(2))), np.array([6.5,3.2,-5.0,-2.5]) )] , 'm' ]
    )

    # Create new spec
    goals = plot_tuples[0][0]
    obs = plot_tuples[1][0]
    
    obs_new = obs
    obs_new.append( (A,b) )

    num_agents = 4
    specs = []
    for i in range(num_agents):
        avoids = [Node('negmu', info={'A':A, 'b':b}) for A, b in obs_new]
        # avoids += [Node('negmu', info={'A':goals[j][0], 'b':goals[j][1]}) for j in range(4) if j is not i]
        avoid_obs = Node('and', deps=avoids)
        always_avoid_obs = Node('A', deps=[avoid_obs,], info={'int':[0,tmax]})
        reach_goal = Node('mu', info={'A':goals[i][0], 'b':goals[i][1]})
        finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0, tmax]})
        specs.append(Node('and', deps=[always_avoid_obs, finally_reach_goal]))

    return plot_tuples1, specs

if __name__ == '__main__':
    # Create Initial Context
    pwl_curves0, plots0, x0s, tmax, specs, bloat, model_data = define_optimistic_scenario()
    num_segs = len(pwl_curves0[0])

    fig1 = plot_plan(pwl_curves0,plots0)
    fig1.savefig("benchmark4-narrow-until_clear.png")

    plots1, specs_real = create_real_context(plots0)

    fig2 = plot_plan(pwl_curves0,plots1)
    fig2.savefig("benchmark4-narrow-until_obstacle.png")

    # Plan for the new context from scratch
    goals = plots1[0][0]
    obs   = plots1[1][0]

    PWLs_replanned,_,_,PWLs_replanned = plan2(
        x0s, specs_real, bloat,
        num_segs=num_segs,
        tmax=tmax,
        ignore_clearance_constraints=True
    )

    # Time Required to Generate One Set of Decision Variables
    smt_repairman = MAPlanRepairman_SMT()
    smt_repairman.add_binary_assignment_constraints(
        x0s, specs_real, bloat, num_segs,
        tmax=tmax
    )
    start = time.time()
    smt_repairman.smt_solver.check()
    end = time.time()

    results = {
        "time-to-replan (full)": PWLs_replanned['Runtime'],
        "time-to-run 1 SMT": end-start
    }

    # Save Some data
    with open('benchmark4-results.yml','w') as outfile:
        yaml.dump(results, outfile, default_flow_style=False)