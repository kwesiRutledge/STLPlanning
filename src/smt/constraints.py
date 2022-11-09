
import sys
sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3

import numpy as np

sys.path.append("../../")
from PWLPlan import Node, _sub, T_MIN_SEP, Conjunction, Disjunction, EPS, M

"""
L1Norm
Description:
    A rewritten version of L1Norm function.
    Useful for the z3 smt solver.
"""
def L1Norm(smt_solver, x):
    # Create Variables
    xvar = []
    abs_x = []
    for dim in range(len(x)):
        xvar.append(
            z3.FreshReal()
        )
        abs_x.append(
            z3.FreshReal()
        )
    
    # Create Constraints
    for i in range(len(x)):
        smt_solver.add(
            xvar[i] == x[i]
        )
        smt_solver.add(
            abs_x[i] == z3.Abs(xvar[i])
        )
    return sum(abs_x)

"""
add_velocity_constraints_smt
Description:
    Creates velocity constraints in terms of the PWLPlan smt variables.
"""
def add_velocity_constraints_smt(smt_solver, PWL, vmax=3):
    for i in range(len(PWL)-1):
        x1, t1 = PWL[i]
        x2, t2 = PWL[i+1]
        # squared_dist = sum([(x1[j]-x2[j])*(x1[j]-x2[j]) for j in range(len(x1))])
        # model.addConstr(squared_dist <= (vmax**2) * (t2 - t1) * (t2 - t1))
        L1_dist = L1Norm(smt_solver, _sub(x1,x2))
        smt_solver.add(
            L1_dist <= vmax * (t2 - t1)
        )

"""
add_time_constraints_smt
Description:
    A copy of the add_time_constraints() function from PWLPlan

"""
def add_time_constraints_smt(smt_solver, PWL, tmax=None):
    if tmax is not None:
        smt_solver.add(
            PWL[-1][1] <= tmax - T_MIN_SEP
        )

    for i in range(len(PWL)-1):
        x1, t1 = PWL[i]
        x2, t2 = PWL[i+1]
        smt_solver.add(
            t2 - t1 >= T_MIN_SEP
        )

"""
clearSpecTree_smt
Description:
"""
def clearSpecTree_smt(spec):
    for dep in spec.deps:
        clearSpecTree_smt(dep)
    spec.zs = []

# """
# handleSpecTree_smt
# Description:
#     Creates constraints based on the nested elements in the
#     specification tree 'spec'.
#     Based on handleSpecTree from PWLPlan.
# """
# def handleSpecTree_smt(spec, PWL, bloat_factor, size):
#     # Recursive call
#     for dep in spec.deps:
#         handleSpecTree(dep, PWL, bloat_factor, size)
    
#     # If all Zs have been assigned, then stop recursing.
#     if len(spec.zs) == len(PWL)-1:
#         return
#     elif len(spec.zs) > 0:
#         raise ValueError('incomplete zs')

#     # Update constraints based on the current node.
#     if spec.op == 'mu':
#         spec.zs = [mu_smt(i, PWL, 0.1, spec.info['A'], spec.info['b']) for i in range(len(PWL)-1)]
#     elif spec.op == 'negmu':
#         spec.zs = [negmu(i, PWL, bloat_factor + size, spec.info['A'], spec.info['b']) for i in range(len(PWL)-1)]
#     elif spec.op == 'and':
#         spec.zs = [Conjunction([dep.zs[i] for dep in spec.deps]) for i in range(len(PWL)-1)]
#     elif spec.op == 'or':
#         spec.zs = [Disjunction([dep.zs[i] for dep in spec.deps]) for i in range(len(PWL)-1)]
#     elif spec.op == 'U':
#         spec.zs = [until(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, spec.deps[1].zs, PWL) for i in range(len(PWL)-1)]
#     elif spec.op == 'F':
#         spec.zs = [eventually(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL) for i in range(len(PWL)-1)]
#     elif spec.op == 'BF':
#         spec.zs = [bounded_eventually(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL, spec.info['tmax']) for i in range(len(PWL)-1)]
#     elif spec.op == 'A':
#         spec.zs = [always(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL) for i in range(len(PWL)-1)]
#     else:
#         raise ValueError('wrong op code')

"""
gen_CDTree_constraints
Description:
    Creates the constraints for the given node 'root'
    by recursively iterating through all constraints of the children of 'root'.
"""
def gen_CDTree_constraints_smt(smt_solver, root):
    if not hasattr(root, 'deps'):
        return [root,]
    else:
        if len(root.constraints)>0:
            # TODO: more check here
            return root.constraints
        dep_constraints = []
        for dep in root.deps:
            dep_constraints.append(gen_CDTree_constraints_smt(smt_solver, dep))
        zs = []
        for dep_con in dep_constraints:
            if isinstance(root, Disjunction):
                z = z3.FreshBool()
                # z = model.addVar(vtype=GRB.BINARY)
                zs.append(z)
                # dep_con = [con + M * (1 - z) for con in dep_con]
                dep_con = [con + M + (-M)*z for con in dep_con]
            root.constraints += dep_con
        if len(zs)>0:
            root.constraints.append(z3.Sum(zs)-1)
        return root.constraints

"""
add_CDTree_Constraints_smt
Description:
    Adds the CD Tree's constraints to the smt_solver for later solution.
"""
def add_CDTree_Constraints_smt(smt_solver, root):
    constrs = gen_CDTree_constraints_smt(smt_solver, root)
    for con in constrs:
        smt_solver.add(
            con >= 0
        )

"""
add_mutual_clearance_constraints_smt
Description:
    Adds mutual clearance constraints for the path in terms of the Z3 solver.
"""
def add_mutual_clearance_constraints_smt(smt_solver:z3.Solver, PWLs, bloat:float):
    raise(
        Exception("This function has not been implemented yet for Z3!")
    )
    for i in range(len(PWLs)):
        for j in range(i+1, len(PWLs)):
            PWL1 = PWLs[i]
            PWL2 = PWLs[j]
            for k in range(len(PWL1)-1):
                for l in range(len(PWL2)-1):
                    x11, t11 = PWL1[k]
                    x12, t12 = PWL1[k+1]
                    x21, t21 = PWL2[l]
                    x22, t22 = PWL2[l+1]
                    z_noIntersection = noIntersection(t11, t12, t21, t22)
                    z_disjoint_segments = disjoint_segments(model, [x11, x12], [x21, x22], bloat)
                    z = Disjunction([z_noIntersection, z_disjoint_segments])
                    add_CDTree_Constraints_smt(model, z)

"""
mu_smt
Description:

"""
def mu_smt(i, PWL, bloat_factor, A, b):
    bloat_factor = np.max([0, bloat_factor])
    # this segment is fully contained in Ax<=b (shrinked)
    b = b.reshape(-1)
    num_edges = len(b)
    conjunctions = []
    for e in range(num_edges):
        a = A[e,:]
        for j in [i, i+1]:
            x = PWL[j][0]
            conjunctions.append(b[e] - np.linalg.norm(a) * bloat_factor - sum([a[k]*x[k] for k in range(len(x))]) - EPS)
    return Conjunction(conjunctions)

"""
plan_smt
Description:
    Implements the planning algorithm in terms of smt variables.
    This function will not only return the final values for the trajectory, but also the values of ALL variables in the Gurobi model.
"""
def plan_smt(x0s, specs, bloat, limits=None, num_segs=None, tasks=None, vmax=3., MIPGap=1e-4, max_segs=None, tmax=None, hard_goals=None, size_list=[],ignore_clearance_constraints=False,add_constraints_on_reals=False):
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
                x_i_a, y_i_a = z3.Reals('x_i^'+str(idx_a)+' y_i^'+str(idx_a))
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

            # Get Size of the object
            size_a = size_list[idx_a]
            handleSpecTree(spec, PWL, bloat, size_a)
            add_CDTree_Constraints(m, spec.zs[0])

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
            add_CDTree_Constraints(m, z)

        if ignore_clearance_constraints:
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