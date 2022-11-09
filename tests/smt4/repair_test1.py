"""
repair_test1.py
Description:

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
    I would just like to give a bad path to the optimization problem and have it verify for me that it is infeasible.
"""
def test1():
    # Constants
    size_list = []
    tmax = 10.0
    repairman0 = MAPlanRepairman_SMT()

    MIPGap = 1e-4

    limits=None
    num_segs=None
    tasks=None
    vmax=3.
    hard_goals=None
    size_list=[]
    ignore_clearance_constraints=False
    add_constraints_on_reals=False

    # Inputs
    pwl_plan, plots = create_optimistic_context()
    
    num_agents = 1

    goals = plots[0][0]
    obs = plots[1][0]
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

    # Algorithm
    num_segs = len(pwl_plan)-1
    x0 = pwl_plan[0][0]
    x0s = [x0]

    # Create the size array, if it does not exist.
    default_size = 0.11*4/2
    if len(size_list) == 0:
        size_list = [default_size for idx_a in range(len(x0s))]

    # Try each segment number between min_segs and max_segs (if it exists)
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
        PWL_z3 = []
        for i in range(num_segs+1):
            PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
        PWLs.append(PWL)
        m.update()

        PWL_z3 = repairman0.copyGurobiPWLToZ3(PWL)
        PWLs_z3.append(PWL_z3)

        # the initial constriant
        m.addConstrs(PWL[0][0][i] == x0[i] for i in range(dims))
        m.addConstr(PWL[0][1] == 0)

        if hard_goals is not None:
            goal = hard_goals[idx_a]
            m.addConstrs(PWL[-1][0][i] == goal[i] for i in range(dims))

        if limits is not None:
            add_space_constraints(m, [P[0] for P in PWL], limits)

        add_velocity_constraints(m, PWL, vmax=vmax)
        add_velocity_constraints_smt(s, PWL_z3, vmax=vmax)

        add_time_constraints(m, PWL, tmax)
        add_time_constraints_smt(s, PWL_z3, tmax)

        
        size_a = size_list[idx_a] # Get Size of the object
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

    m.write("plan-simultaneous.lp")
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
test2
Description:
    I would just like to give a bad path to the optimization problem and have it 
    verify for me that it is infeasible.
"""
def test2():
    # Constants
    size_list = []
    tmax = 10.0
    repairman0 = MAPlanRepairman_SMT()

    MIPGap = 1e-4

    limits=None
    num_segs=None
    tasks=None
    vmax=3.
    hard_goals=None
    size_list=[]
    ignore_clearance_constraints=False
    add_constraints_on_reals=False

    # Inputs
    pwl_plan, plots = create_optimistic_context()
    
    num_agents = 1

    goals = plots[0][0]
    obs = plots[1][0]
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

    # Algorithm
    num_segs = len(pwl_plan)-1
    x0 = pwl_plan[0][0]
    x0s = [x0]

    # Create the size array, if it does not exist.
    default_size = 0.11*4/2
    if len(size_list) == 0:
        size_list = [default_size for idx_a in range(len(x0s))]

    # Try each segment number between min_segs and max_segs (if it exists)
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
        PWL_z3 = []
        for i in range(num_segs+1):
            PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
        PWLs.append(PWL)
        m.update()

        PWL_z3 = repairman0.copyGurobiPWLToZ3(PWL)
        PWLs_z3.append(PWL_z3)

        # the initial constriant
        m.addConstrs(PWL[0][0][i] == x0[i] for i in range(dims))
        m.addConstr(PWL[0][1] == 0)

        if hard_goals is not None:
            goal = hard_goals[idx_a]
            m.addConstrs(PWL[-1][0][i] == goal[i] for i in range(dims))

        if limits is not None:
            add_space_constraints(m, [P[0] for P in PWL], limits)

        add_velocity_constraints(m, PWL, vmax=vmax)
        add_velocity_constraints_smt(s, PWL_z3, vmax=vmax)

        add_time_constraints(m, PWL, tmax)
        add_time_constraints_smt(s, PWL_z3, tmax)

        
        size_a = size_list[idx_a] # Get Size of the object
        repairman0.handleSpecTree(spec,    PWL,   bloat, size_a)
        repairman0.handleSpecTree(spec_z3, PWL_z3,bloat, size_a)
        repairman0.add_CDTree_Constraints(m, s, spec.zs[0] , spec_z3.zs[0] )

        # Compute Constraints for The Path
        for waypoint_index in range(len(PWL)):
            P_star = pwl_plan[waypoint_index]
            P      = PWL[waypoint_index]

            m.addConstrs(
                P_star[0][i] == P[0][i] for i in range(dims)
            )

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

    m.write("plan-simultaneous.lp")
    print('NumBinVars: %d'%m.getAttr('NumBinVars'))

    start = time.time()
    m.optimize()
    end = time.time()
    print('solving it takes %.3f s'%(end - start))

    return end-start, m.getAttr('Status')
    

"""
test3
Description:
    I would just like to give a bad path to the optimization problem and have it 
    verify for me that it is infeasible.
    Using function for this now.
"""
def test3():
    # Constants
    size_list = []
    tmax = 10.0
    repairman0 = MAPlanRepairman_SMT()

    bloat = 0.1

    # Inputs
    pwl_plan, plots = create_optimistic_context()
    
    goals = plots[0][0]
    obs = plots[1][0]
    specs = []
    for i in range(1):
        avoids = [Node('negmu', info={'A':A, 'b':b}) for A, b in obs]
        print(avoids)
        # avoids += [Node('negmu', info={'A':goals[j][0], 'b':goals[j][1]}) for j in range(4) if j is not i]
        avoid_obs = Node('and', deps=avoids)
        always_avoid_obs = Node('A', deps=[avoid_obs,], info={'int':[0,tmax]})
        reach_goal = Node('mu', info={'A':goals[i][0], 'b':goals[i][1]})
        finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0, tmax]})
        specs.append(Node('and', deps=[always_avoid_obs, finally_reach_goal]))

    return repairman0.check_plan([pwl_plan],specs,bloat,tmax=tmax)

if __name__ == '__main__':
    # Constants

    # Algorithm
    _,_,_,model_info1 = test1()
    timing2,status2 = test2()
    timing3,status3 = test3()

    test_data = {
        "test1 full replanning struct":test1(),
        "test2":{"timing":timing2,"status":status2 },
        "test3":{"timing":timing3,"status":status3 }
    }

    # Save Results
    with open('smt-repair-tests.yml','w') as outfile:
        yaml.dump(test_data, outfile, default_flow_style=False)

