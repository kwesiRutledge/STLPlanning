"""
simultaneous_test1.py
Description:

"""
from gurobipy import *
import sys, yaml, copy, time

import numpy as np

sys.path.append("../../")
from src.smt.simultaneous_synthesis import MAPlanRepairman_SMT
from PWLPlan import Node, IntFeasTol, add_space_constraints, add_time_constraints, add_velocity_constraints, add_mutual_clearance_constraints
from src.smt.constraints import add_time_constraints_smt, add_velocity_constraints_smt

sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3

def test1():
    # Attempt to create
    try:
        repairman0 = MAPlanRepairman_SMT()
        return True
    except:
        return False

"""
test2
Description:
    Testing how well we can extract the names of gurobi variables.
"""
def test2():

    # Constants
    repairman0 = MAPlanRepairman_SMT()

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
            repairman0.clearSpecTree(spec)
        print('----------------------------')
        print('num_segs', num_segs)

        PWLs = []
        m = Model("xref")
        m.setParam(GRB.Param.OutputFlag, 0)
        m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
        m.setParam(GRB.Param.MIPGap, MIPGap)
        # m.setParam(GRB.Param.NonConvex, 2)
        # m.getEnv().set(GRB_IntParam_OutputFlag, 0)

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
            PWLs.append(PWL)
            m.update()
        
            for waypoint_i in PWL:
                if waypoint_i[0][0].VarName == waypoint_i[0][1].VarName:
                    return False
                if waypoint_i[0][0].VarName == waypoint_i[1].VarName:
                    return False
                if waypoint_i[0][1].VarName == waypoint_i[1].VarName:
                    return False

    return True

"""
test2b
Description:
    Testing how well we can COPY the names of gurobi variables.
"""
def test2b():

    # Constants
    repairman0 = MAPlanRepairman_SMT()

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
            repairman0.clearSpecTree(spec)
        print('----------------------------')
        print('num_segs', num_segs)

        PWLs = []
        m = Model("xref")
        m.setParam(GRB.Param.OutputFlag, 0)
        m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
        m.setParam(GRB.Param.MIPGap, MIPGap)
        # m.setParam(GRB.Param.NonConvex, 2)
        # m.getEnv().set(GRB_IntParam_OutputFlag, 0)

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
            PWLs.append(PWL)
            m.update()
        
            PWL_z3 = repairman0.copyGurobiPWLToZ3(PWL)

            for i in range(len(PWL)):
                z3_waypoint_i = PWL_z3[i]
                gurobi_waypoint_i = PWL[i]

                # Check names
                if str(z3_waypoint_i[0][0]) != gurobi_waypoint_i[0][0].VarName:
                    return False

                if str(z3_waypoint_i[0][1]) != gurobi_waypoint_i[0][1].VarName:
                    return False

                if str(z3_waypoint_i[1]) != gurobi_waypoint_i[1].VarName:
                    return False

    return True

"""
test3
Description:
    Testing how well we can COPY the names of gurobi variables using a built-in function for 
    the repairman.
"""
def test3():

    # Constants
    repairman0 = MAPlanRepairman_SMT()

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
            repairman0.clearSpecTree(spec)

        print('----------------------------')
        print('num_segs', num_segs)

        PWLs = []
        PWLs_z3 = []
        m = Model("xref")
        m.setParam(GRB.Param.OutputFlag, 0)
        m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
        m.setParam(GRB.Param.MIPGap, MIPGap)
        # m.setParam(GRB.Param.NonConvex, 2)
        # m.getEnv().set(GRB_IntParam_OutputFlag, 0)

        # Create z3 solver
        s = z3.Solver()

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]
            spec_z3 = copy.deepcopy(spec)

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
            PWLs.append(PWL)
            m.update()

            PWL_z3 = repairman0.copyGurobiPWLToZ3(PWL)
            PWLs_z3.append(PWL_z3)

            # the initial constriant
            m.addConstrs(PWL[0][0][i] == x0[i] for i in range(dims))
            m.addConstr(PWL[0][1] == 0)

            # the initial constriant (for Z3)
            s.add([PWL_z3[0][0][i] == x0[i] for i in range(dims)])
            s.add(PWL_z3[0][1] == 0.0)

            if hard_goals is not None:
                goal = hard_goals[idx_a]
                m.addConstrs(PWL[-1][0][i] == goal[i] for i in range(dims))
                s.add(PWL_z3[-1][0][i] == goal[i] for i in range(dims))

            if limits is not None:
                add_space_constraints(m, [P[0] for P in PWL], limits)

            add_velocity_constraints(m, PWL, vmax=vmax)
            add_velocity_constraints_smt(s, PWL_z3, vmax=vmax)

            add_time_constraints(m, PWL, tmax)
            add_time_constraints_smt(s, PWL_z3, tmax)

            # Get Size of the object
            size_a = size_list[idx_a]
            repairman0.handleSpecTree(spec,    PWL,   bloat, size_a)
            repairman0.handleSpecTree(spec_z3, PWL_z3,bloat, size_a)
            print("spec.zs = ",spec.zs)
            print(spec.zs[0])
            repairman0.add_CDTree_Constraints(m, s, spec.zs[0] , spec_z3.zs[0] )

    return len(spec.zs[0].constraints), len(spec_z3.zs[0].constraints)

"""
test4
Description:
    Testing how well we can add the rest of the planning constraints without errors..
"""
def test4():

    # Constants
    repairman0 = MAPlanRepairman_SMT()

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
            repairman0.clearSpecTree(spec)

        print('----------------------------')
        print('num_segs', num_segs)

        PWLs = []
        PWLs_z3 = []
        m = Model("xref")
        m.setParam(GRB.Param.OutputFlag, 0)
        m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
        m.setParam(GRB.Param.MIPGap, MIPGap)
        # m.setParam(GRB.Param.NonConvex, 2)
        # m.getEnv().set(GRB_IntParam_OutputFlag, 0)

        # Create z3 solver
        s = z3.Solver()

        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]
            spec_z3 = copy.deepcopy(spec)

            dims = len(x0)

            PWL = []
            for i in range(num_segs+1):
                PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
            PWLs.append(PWL)
            m.update()

            PWL_z3 = repairman0.copyGurobiPWLToZ3(PWL)
            PWLs_z3.append(PWL_z3)

            # the initial constriant
            m.addConstrs(PWL[0][0][i] == x0[i] for i in range(dims))
            m.addConstr(PWL[0][1] == 0)

            # the initial constriant (for Z3)
            s.add([PWL_z3[0][0][i] == x0[i] for i in range(dims)])
            s.add(PWL_z3[0][1] == 0.0)

            if hard_goals is not None:
                goal = hard_goals[idx_a]
                m.addConstrs(PWL[-1][0][i] == goal[i] for i in range(dims))
                s.add(PWL_z3[-1][0][i] == goal[i] for i in range(dims))

            if limits is not None:
                add_space_constraints(m, [P[0] for P in PWL], limits)

            add_velocity_constraints(m, PWL, vmax=vmax)
            add_velocity_constraints_smt(s, PWL_z3, vmax=vmax)

            add_time_constraints(m, PWL, tmax)
            add_time_constraints_smt(s, PWL_z3, tmax)

            # Get Size of the object
            size_a = size_list[idx_a]
            repairman0.handleSpecTree(spec,    PWL,   bloat, size_a)
            repairman0.handleSpecTree(spec_z3, PWL_z3,bloat, size_a)
            repairman0.add_CDTree_Constraints(m, s, spec.zs[0] , spec_z3.zs[0] )

        if tasks is not None:
            for idx_agent in range(len(tasks)):
                size_a = size_list[idx_agent]
                for idx_task in range(len(tasks[idx_agent])):
                    repairman0.handleSpecTree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size_a)
                    repairman0.handleSpecTree(
                        copy.deepcopy(tasks[idx_agent][idx_task]), PWLs_z3[idx_agent], bloat, size_a
                    )

            conjunctions = []
            for idx_task in range(len(tasks[0])):
                disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                conjunctions.append(self.Disjunction(disjunctions))
            z = repairman0.Conjunction(conjunctions)
            repairman0.add_CDTree_Constraints(m, s, z, copy.deepcopy(z))

        if not ignore_clearance_constraints:
            add_mutual_clearance_constraints(m, PWLs, bloat)

        # obj = sum([L1Norm(m, _sub(PWL[i][0], PWL[i+1][0])) for PWL in PWLs for i in range(len(PWL)-1)])
        obj = sum([PWL[-1][1] for PWL in PWLs])
        m.setObjective(obj, GRB.MINIMIZE)

        m.write("plan2.lp")
        print('NumBinVars: %d'%m.getAttr('NumBinVars'))

        # m.computeIIS()
        # import ipdb;ipdb.set_treace()
        try:
            start = time.time()
            m.optimize()
            end = time.time()
            print('solving it takes %.3f s'%(end - start))
            PWLs_output = []
            for PWL in PWLs:
                PWL_output = []
                for P in PWL:
                    PWL_output.append([[P[0][i].X for i in range(len(P[0]))], P[1].X])
                PWLs_output.append(PWL_output)

            # Extract model data
            # - all model variables
            vars = m.getVars()
            var_values = [var.X for var in vars]
            # - runtime
            model_data = {
                'Runtime': m.Runtime,
                'TypeList': [var.VType for var in vars]
            }

            m.dispose()
            return PWLs_output, vars, var_values, model_data
        except Exception as e:
            print("Exception detected: " + str(e))
            m.dispose()
    return [None,None,None,None]

"""
test5
Description:
    Testing how well we can add the rest of the planning constraints when everything is embedded in a function.
"""
def test5():

    # Constants
    repairman0 = MAPlanRepairman_SMT()

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

    return repairman0.plan(
        x0s, specs, bloat, 
        num_segs=num_segs, MIPGap=MIPGap
    )

    

    

if __name__ == '__main__':
    # Announce Start
    print("simultaneous_test1.py")
    print("")

    test_data = {
        "constructor works?":test1(),
        "Getting names works?":test2(),
        "Copying names works?":test2b(),
        "# of constraints for (MILP v. SMT)":test3(),
        "Planner Output": test4(),
        "Planner (Functionalized) Output": test5()
    }

    print(test_data)

    # Save Results
    with open('smt-planner-tests.yml','w') as outfile:
        yaml.dump(test_data, outfile, default_flow_style=False)