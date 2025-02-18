import sys
import numpy as np

from PWLPlan import plan, Node
from vis import vis

def test_clear():
    x0s = [[2, 0.5], [8, 0.5]]
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
    goals = [(A, b1), (A, b2) ]

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

    PWL = plan(x0s, specs, bloat=0.2, num_segs=6, tmax=tmax, vmax=vmax, MIPGap = 0.3)

    plots = [[goals, 'g'], [obs, 'k']]
    return x0s, plots, PWL

def test_obstruction():
    x0s = [[2, 0.5], [8, 0.5]]
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

    # Add extra avoid region.
    A_extra = np.vstack( (np.eye(2) , -np.eye(2)) )
    b_extra = np.array([6.3, 3.8, -5.3, -3.0])
    # avoid_extra_obstacle = Node('negmu', info={'A':A_extra, 'b':b_extra})
    obs.append((A_extra,b_extra))

    b1 = np.array([-1.5, 2.5, -3, 4], dtype = np.float64)
    b2 = np.array([-7.5, 8.5, -3, 4], dtype = np.float64)
    b3 = np.array([-1.5, 2.5, 0, 1], dtype = np.float64)
    b4 = np.array([-7.5, 8.5, 0, 1], dtype = np.float64)
    goals = [(A, b1), (A, b2)]

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

    PWL = plan(x0s, specs, bloat=0.2, num_segs=7, tmax=tmax, vmax=vmax, MIPGap = 0.3)

    plots = [[goals, 'g'], [obs, 'k']]
    return x0s, plots, PWL

if __name__ == '__main__':
    filename1 = "wall-3-clear.png"
    filename2 = "wall-3-obstruct.png"
    results1 = vis(test_clear,filename="images/"+filename1)
    results2 = vis(test_obstruction,filename="images/"+filename2)

