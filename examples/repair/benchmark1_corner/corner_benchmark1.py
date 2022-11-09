"""
corner_benchmark-1.py
Description:
    Creates code for the first benchmark problem.
    It involves a single agent moving around a single corner.
"""

import numpy as np
import sys,time,yaml
sys.path.append('../../../')
from PWLPlan import plan2, Node
from src.plot_plan import plot_plan
from src.smt.simultaneous_synthesis import MAPlanRepairman_SMT


"""
create_optimistic_context
Description:
    Creates the following values for the corner_benchmark #1
    - PWL plan (for 1 agent)
    - Obstacle Free Environment
"""
def create_optimistic_context():

    # Create Initial Path
    pwl_curve = [
        (np.array([1.0,1.0]),0.0),
        (np.array([1.25,1.25]),2.0),
        (np.array([1.5,1.5]),4.0),
        (np.array([1.75,1.75]),6.0),
        (np.array([2.0,2.0]),7.0),
        (np.array([2.25,2.25]),8.0),
        (np.array([2.5,2.5]),10.0)
    ] # Example Curve

    x0s = [
        pwl_curve[0][0]
    ]

    tmax = 10.0

    # Create walls around workspace
    wall_half_width = 0.05
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    walls = []

    walls.append(np.array([0, 0, 0, 4], dtype = np.float64))
    walls.append(np.array([5, 5, 0, 4], dtype = np.float64))
    walls.append(np.array([0, 5, 0, 0], dtype = np.float64))
    walls.append(np.array([0, 5, 4, 4], dtype = np.float64))

    # walls.append(np.array([0, 4., 2, 2], dtype = np.float64))
    # walls.append(np.array([5., 10, 2, 2], dtype = np.float64))

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

    b1 = np.array([-2.25, 2.75, -2.3, 2.8], dtype = np.float64)
    goals = [(A, b1)]

    plots = [[goals, 'g'], [obs, 'k']]

    num_agents = len(x0s)
    specs = []
    for i in range(num_agents):
        avoids = [Node('negmu', info={'A':A, 'b':b}) for A, b in obs]
        print(avoids)
        # avoids += [Node('negmu', info={'A':goals[j][0], 'b':goals[j][1]}) for j in range(4) if j is not i]
        avoid_obs = Node('and', deps=avoids)
        always_avoid_obs = Node('A', deps=[avoid_obs,], info={'int':[0,tmax]})
        reach_goal = Node('mu', info={'A':goals[i][0], 'b':goals[i][1]})
        finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0, tmax]})
        specs.append(Node('and', deps=[always_avoid_obs, finally_reach_goal]))

    bloat = 0.1

    return pwl_curve, plots, x0s, tmax, specs, bloat

"""
create_real_context
Description:
    Create context
"""
def create_real_context(plot_tuples):
    # Constants
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])

    # Extract obs from the plot tuples list
    plot_tuples1 = plot_tuples.copy()
    b = np.array([-1.5, 2.2, -0.5, 2.2], dtype = np.float64)
    plot_tuples1.append(
        [ [(A,b)] , 'm' ]
    )

    # Create new spec
    goals = plot_tuples[0][0]
    obs = plot_tuples[1][0]

    obs_new = obs
    obs_new.append( (A,b) )

    num_agents = 1
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
    pwl_curve0, plots0, x0s, tmax, specs, bloat = create_optimistic_context()
    num_segs = len(pwl_curve0)-1

    fig1 = plot_plan([pwl_curve0],plots0)
    fig1.savefig("corner_benchmark-1-clear.png")

    plots1, specs_real = create_real_context(plots0)

    fig2 = plot_plan([pwl_curve0],plots1)
    fig2.savefig("corner_benchmark-1-obstacle.png")

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

    # Save Results
    with open('benchmark1-results.yml','w') as outfile:
        yaml.dump(results, outfile, default_flow_style=False)
