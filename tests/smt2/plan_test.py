"""
plan_test.py
Description:
    Tests the ability of our code to 
"""

import sys
sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3

import numpy as np
import yaml, time

sys.path.append("../../")
from PWLPlan import Node, handleSpecTree
from src.smt.constraints import clearSpecTree_smt, add_velocity_constraints_smt, add_time_constraints_smt, add_CDTree_Constraints_smt

"""
test1
Description:
    Tests the first chunk of plan_smt.
"""
def test1():
    # Constants
    limits=None
    num_segs=None
    tasks=None
    vmax=3.
    MIPGap=1e-4
    max_segs=None
    tmax=None
    hard_goals=None
    size_list=[]
    ignore_clearance_constraints=False
    add_constraints_on_reals=True

    # Inputs

    x0s = [
        np.array([0.0,0.5]),
        np.array([0.5,-0.5]),
        np.array([-0.5,-0.5])
    ]

    num_agents = len(x0s)

    agent_radius = 0.22

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
    tmax=10.0
    vmax=3.0
    MIPGap = 0.3

    # Algorithm

    # Create the limitations on the number of piece-wise linear segments.
    if num_segs is None:
        min_segs = 1
        assert max_segs is not None
    else:
        min_segs = num_segs
        max_segs = num_segs

    # Create the size array, if it does not exist.
    default_size = 0.11*4/2
    if len(size_list) == 0:
        size_list = [default_size for idx_a in range(len(x0s))]

    # Try each segment number between min_segs and max_segs (if it exists)
    for num_segs in range(min_segs, max_segs+1):
        for spec in specs:
            clearSpecTree_smt(spec)
        print('----------------------------')
        print('num_segs', num_segs)

        s = z3.Solver()

        PWLs = []

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                x_i_a, y_i_a = z3.Reals('x_i^'+str(idx_a)+' y_i^' +str(idx_a))
                PWL.append([ [x_i_a,y_i_a] , z3.Real('t_i^'+str(idx_a))])
            PWLs.append(PWL)

            if add_constraints_on_reals:
                # the initial constriant
                s.add(
                    [PWL[0][0][i] == x0[i] for i in range(dims)]
                )
                s.add(
                    PWL[0][1] == 0.0
                )

                if hard_goals is not None:
                    goal = hard_goals[idx_a]
                    s.add(
                        PWL[-1][0][i] == goal[i] for i in range(dims)
                    )

                if limits is not None:
                    add_space_constraints(m, [P[0] for P in PWL], limits)

                add_velocity_constraints_smt(s, PWL, vmax=vmax)
                add_time_constraints_smt(s, PWL, tmax)

    return True

"""
test2
Description:
    Tests the second chunk of plan_smt.
    Should contain handle_specTree. Pray for me.
"""
def test2():
    # Constants
    limits=None
    num_segs=None
    tasks=None
    vmax=3.
    MIPGap=1e-4
    max_segs=None
    tmax=None
    hard_goals=None
    size_list=[]
    ignore_clearance_constraints=False
    add_constraints_on_reals=False

    # Inputs

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

    # Algorithm

    # Create the limitations on the number of piece-wise linear segments.
    if num_segs is None:
        min_segs = 1
        assert max_segs is not None
    else:
        min_segs = num_segs
        max_segs = num_segs

    # Create the size array, if it does not exist.
    default_size = 0.11*4/2
    if len(size_list) == 0:
        size_list = [default_size for idx_a in range(len(x0s))]

    # Try each segment number between min_segs and max_segs (if it exists)
    for num_segs in range(min_segs, max_segs+1):
        for spec in specs:
            clearSpecTree_smt(spec)
        print('----------------------------')
        print('num_segs', num_segs)

        s = z3.Solver()

        PWLs = []

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                x_i_a, y_i_a = z3.Reals('x_'+str(i)+'^'+str(idx_a)+' y_'+str(i)+'^' +str(idx_a))
                PWL.append([ [x_i_a,y_i_a] , z3.Real('t_'+str(i)+'^'+str(idx_a))])
            PWLs.append(PWL)
            
            print(PWL)

            if add_constraints_on_reals:
                # the initial constriant
                s.add(
                    [PWL[0][0][i] == x0[i] for i in range(dims)]
                )
                s.add(
                    PWL[0][1] == 0.0
                )

                if hard_goals is not None:
                    goal = hard_goals[idx_a]
                    s.add(
                        PWL[-1][0][i] == goal[i] for i in range(dims)
                    )

                if limits is not None:
                    add_space_constraints(m, [P[0] for P in PWL], limits)

                add_velocity_constraints_smt(s, PWL, vmax=vmax)
                add_time_constraints_smt(s, PWL, tmax)

            # Get Size of the object
            size_a = size_list[idx_a]
            handleSpecTree(spec, PWL, bloat, size_a)
            add_CDTree_Constraints_smt(s, spec.zs[0])

        if tasks is not None:
            for idx_agent in range(len(tasks)):
                size_a = size_list[idx_agent]
                for idx_task in range(len(tasks[idx_agent])):
                    handleSpecTree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size_a)

            conjunctions = []
            for idx_task in range(len(tasks[0])):
                disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                conjunctions.append(Disjunction(disjunctions))
            z = Conjunction(conjunctions)
            add_CDTree_Constraints_smt(s, z)

    return True

"""
test3
Description:
    Tests the complete plan_smt algorithm.
    We've completed most of the SpecTree stuff!
"""
def test3():
    # Constants
    limits=None
    num_segs=None
    tasks=None
    vmax=3.
    MIPGap=1e-4
    max_segs=None
    tmax=None
    hard_goals=None
    size_list=[]
    ignore_clearance_constraints=False
    add_constraints_on_reals=False

    # Inputs

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

    # Algorithm

    # Create the limitations on the number of piece-wise linear segments.
    if num_segs is None:
        min_segs = 1
        assert max_segs is not None
    else:
        min_segs = num_segs
        max_segs = num_segs

    # Create the size array, if it does not exist.
    default_size = 0.11*4/2
    if len(size_list) == 0:
        size_list = [default_size for idx_a in range(len(x0s))]

    # Try each segment number between min_segs and max_segs (if it exists)
    for num_segs in range(min_segs, max_segs+1):
        for spec in specs:
            clearSpecTree_smt(spec)
        print('----------------------------')
        print('num_segs', num_segs)

        s = z3.Solver()

        PWLs = []

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                x_i_a, y_i_a = z3.Reals('x_'+str(i)+'^'+str(idx_a)+' y_'+str(i)+'^' +str(idx_a))
                PWL.append([ [x_i_a,y_i_a] , z3.Real('t_'+str(i)+'^'+str(idx_a))])
            PWLs.append(PWL)
            
            #print(PWL)

            if add_constraints_on_reals:
                # the initial constriant
                s.add(
                    [PWL[0][0][i] == x0[i] for i in range(dims)]
                )
                s.add(
                    PWL[0][1] == 0.0
                )

                if hard_goals is not None:
                    goal = hard_goals[idx_a]
                    s.add(
                        PWL[-1][0][i] == goal[i] for i in range(dims)
                    )

                if limits is not None:
                    add_space_constraints(m, [P[0] for P in PWL], limits)

                add_velocity_constraints_smt(s, PWL, vmax=vmax)
                add_time_constraints_smt(s, PWL, tmax)

            # Get Size of the object
            size_a = size_list[idx_a]
            handleSpecTree(spec, PWL, bloat, size_a)
            add_CDTree_Constraints_smt(s, spec.zs[0])

        if (tasks is not None) and add_constraints_on_reals:
            for idx_agent in range(len(tasks)):
                size_a = size_list[idx_agent]
                for idx_task in range(len(tasks[idx_agent])):
                    handleSpecTree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size_a)

            conjunctions = []
            for idx_task in range(len(tasks[0])):
                disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                conjunctions.append(Disjunction(disjunctions))
            z = Conjunction(conjunctions)
            add_CDTree_Constraints_smt(s, z)

        if add_constraints_on_reals:
            add_mutual_clearance_constraints_smt(m, PWLs, bloat)

        # obj = sum([L1Norm(m, _sub(PWL[i][0], PWL[i+1][0])) for PWL in PWLs for i in range(len(PWL)-1)])
        # obj = sum([PWL[-1][1] for PWL in PWLs])
        # m.setObjective(obj, GRB.MINIMIZE)

        # Solve
        start = time.time()
        s.check()
        end = time.time()
        print('solving it takes %.3f s'%(end - start))
        print(
            s.model()
        )

    return True

if __name__ == '__main__':
    # Announce Start
    print("plan_test.py")
    print("")

    test_data = {
        "test1":test1(),
        "test2":test2(),
        "test3":test3()
    }

    # Save Results
    with open('smt-planner-tests.yml','w') as outfile:
        yaml.dump(test_data, outfile, default_flow_style=False)
