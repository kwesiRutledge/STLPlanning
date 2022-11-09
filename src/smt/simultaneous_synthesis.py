"""
simultaneous_synthesis
Description:

"""

import time, sys, copy
import numpy as np

from gurobipy import *

sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3

sys.path.append('../../')
from PWLPlan import mu, negmu, always, until, eventually, bounded_eventually, \
                    EPS, M, handleSpecTree, Node, add_time_constraints, add_velocity_constraints, \
                    IntFeasTol, add_mutual_clearance_constraints
from src.smt.constraints import add_velocity_constraints_smt, add_time_constraints_smt

class MAPlanRepairman_SMT():
    def __init__(self) -> None:
        self.smt_solver = []
        pass

    """
    Conjunction (Inner Class)
    Description:

    """
    class Conjunction(object):
        # conjunction node
        def __init__(self, deps = [], z3_deps=[]):
            super(MAPlanRepairman_SMT.Conjunction, self).__init__()
            self.deps = deps
            self.z3_deps = z3_deps
            self.constraints = []
            self.z3_constraints = []

        def __str__(self) -> str:
            return "Conjunction with:\n- deps = " + \
                str(self.deps) + "\n- z3_deps = " + \
                str(self.z3_deps) + "\n- constraints = " + \
                str(self.constraints)

    class Disjunction(object):
        # disjunction node
        def __init__(self, deps = [], z3_deps=[]):
            super(MAPlanRepairman_SMT.Disjunction, self).__init__()
            self.deps = deps
            self.z3_deps = z3_deps
            self.constraints = []
            self.z3_constraints = []

    # class Node(object):
    #     """docstring for Node"""
    #     def __init__(self, op, deps = [], z3_deps=[], zs = [], info = []):
    #         super(MAPlanRepairman_SMT.Node, self).__init__()
    #         self.op = op
    #         self.deps = deps
    #         self.z3_deps = z3_deps
    #         self.zs = zs
    #         self.info = info

    """
    clearSpecTree
    Description:
        Clears all potential dep
    """
    def clearSpecTree(self,spec):
        for dep in spec.deps:
            self.clearSpecTree(dep)
        spec.zs = []

    """
    copyGurobiPWLToZ3:
    Description:
        Returns a PWL similar to PWL_in, but instead of the variables of PWL_in being Gurobi "Var"
        objects, they are Z3 variables.
    """
    def copyGurobiPWLToZ3(self,PWL_in:list[tuple[list[Var],Var]]):
        # Constants

        # Algorithm
        PWL_out = []
        for waypoint_i in PWL_in:
            PWL_out.append(
                ( [ z3.Real( waypoint_i[0][0].VarName ) , z3.Real( waypoint_i[0][1].VarName ) ] , z3.Real( waypoint_i[1].VarName ) )
            )

        return PWL_out

    """
    handleSpecTree()
    Description:

    """
    def handleSpecTree(self,spec, PWL, bloat_factor, size):
        # Recursive Call
        for dep in spec.deps:
            self.handleSpecTree(dep, PWL, bloat_factor, size)
        
        # Checking If SpecTree is already assembled
        if len(spec.zs) == len(PWL)-1:
            return
        elif len(spec.zs) > 0:
            raise ValueError('incomplete zs')
        
        # Iterating through each of the constraint functions.
        if spec.op == 'mu':
            spec.zs    = [self.mu(i, PWL, 0.1, spec.info['A'], spec.info['b']) for i in range(len(PWL)-1)]
        elif spec.op == 'negmu':
            spec.zs    = [self.negmu(i, PWL, bloat_factor + size, spec.info['A'], spec.info['b']) for i in range(len(PWL)-1)]
        elif spec.op == 'and':
            spec.zs    = [self.Conjunction([dep.zs[i] for dep in spec.deps]) for i in range(len(PWL)-1)]
        elif spec.op == 'or':
            spec.zs    = [self.Disjunction([dep.zs[i] for dep in spec.deps]) for i in range(len(PWL)-1)]
        elif spec.op == 'U':
            spec.zs    = [until(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, spec.deps[1].zs, PWL) for i in range(len(PWL)-1)]
        elif spec.op == 'F':
            spec.zs    = [self.eventually(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL) for i in range(len(PWL)-1)]
        elif spec.op == 'BF':
            spec.zs    = [bounded_eventually(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL, spec.info['tmax']) for i in range(len(PWL)-1)]
        elif spec.op == 'A':
            spec.zs    = [self.always(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL) for i in range(len(PWL)-1)]
        else:
            raise ValueError('wrong op code')

    """
    gen_CDTree_constraints
    Description:

    """
    def gen_CDTree_constraints(self, model, smt_solver, root, z3_root):
        if not hasattr(root, 'deps'):
            return [root,],[z3_root,]
        else:
            if len(root.constraints)>0:
                # TODO: more check here
                return root.constraints, z3_root.constraints
            dep_constraints = []
            dep_z3_constraints = []
            for dep_index in range(len(root.deps)):
                dep = root.deps[dep_index]
                z3_dep = z3_root.deps[dep_index]

                temp_cons, temp_z3_cons = self.gen_CDTree_constraints(model, smt_solver, dep, z3_dep)
                dep_constraints.append(temp_cons)
                dep_z3_constraints.append(temp_z3_cons)
            zs = []
            zs_z3 = []
            for dep_con_index in range(len(dep_constraints)):
                dep_con = dep_constraints[dep_con_index]
                dep_z3_con = dep_z3_constraints[dep_con_index]
                if isinstance(root, self.Disjunction):
                    z = model.addVar(vtype=GRB.BINARY)
                    model.update()
                    zs.append(z)

                    z_z3 = z3.Bool(z.VarName)
                    zs_z3.append(z_z3)

                    # Creating dependent constraints
                    dep_con = [con + M * (1 - z) for con in dep_con]
                    dep_z3_con = [con + M + (-M)*z_z3 for con in dep_z3_con]
                root.constraints += dep_con
                z3_root.constraints += dep_z3_con
            if len(zs)>0:
                root.constraints.append(sum(zs)-1)
                root.z3_constraints.append(sum(zs)-1)
            model.update()
            return root.constraints, z3_root.constraints

    """
    add_CDTree_Constraints
    Description:

    """
    def add_CDTree_Constraints(self, model, smt_solver, root, z3_root):
        constrs, z3_constrs = self.gen_CDTree_constraints(model, smt_solver, root, z3_root)
        # assert( len(constrs) == len(z3_constrs) )
        for con in constrs:
            model.addConstr(con >= 0)

        for con in z3_constrs:
            smt_solver.add(con >= 0)

    """
    plan
    Description:
        Introducing a breaking change to plan.
        This function will not only return the final values for the trajectory, but also the values of ALL variables in the Gurobi model.
    """
    def plan(self, x0s, specs, bloat, limits=None, num_segs=None, tasks=None, vmax=3., MIPGap=1e-4, max_segs=None, tmax=None, hard_goals=None, size_list=[],ignore_clearance_constraints=False):
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
                self.clearSpecTree(spec)
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

                PWL_z3 = self.copyGurobiPWLToZ3(PWL)
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

                # Get Size of the object
                size_a = size_list[idx_a]
                self.handleSpecTree(spec,    PWL,   bloat, size_a)
                self.handleSpecTree(spec_z3, PWL_z3,bloat, size_a)
                self.add_CDTree_Constraints(m, s, spec.zs[0] , spec_z3.zs[0] )

            if tasks is not None:
                for idx_agent in range(len(tasks)):
                    size_a = size_list[idx_agent]
                    for idx_task in range(len(tasks[idx_agent])):
                        self.handleSpecTree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size_a)
                        self.handleSpecTree(
                            copy.deepcopy(tasks[idx_agent][idx_task]), PWLs_z3[idx_agent], bloat, size_a
                        )

                conjunctions = []
                for idx_task in range(len(tasks[0])):
                    disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                    conjunctions.append(self.Disjunction(disjunctions))
                z = self.Conjunction(conjunctions)
                self.add_CDTree_Constraints(m, s, z, copy.deepcopy(z))

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
    check_plan
    Description:
        Checks whether or not a given plan satisfies the specification given.
    """
    def check_plan(self, PWLs_in,specs,bloat,MIPGap=1e-4,size_list=[],tmax=None,limits=None,tasks=None,vmax=3.,hard_goals=None,ignore_clearance_constraints=True):
        # Constants
        num_segs = len(PWLs_in[0])-1
        num_agents = len(PWLs_in)

        x0s = []
        for idx_a in range(num_agents):
            PWL_a = PWLs_in[idx_a]
            x0s.append( PWL_a[0][0] )

        # Algorithm

        # Create the size array, if it does not exist.
        default_size = 0.11*4/2
        if len(size_list) == 0:
            size_list = [default_size for idx_a in range(len(x0s))]

        # Try each segment number between min_segs and max_segs (if it exists)
        for spec in specs:
            self.clearSpecTree(spec)
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

            PWL_z3 = self.copyGurobiPWLToZ3(PWL)
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
            self.handleSpecTree(spec,    PWL,   bloat, size_a)
            self.handleSpecTree(spec_z3, PWL_z3,bloat, size_a)
            self.add_CDTree_Constraints(m, s, spec.zs[0] , spec_z3.zs[0] )

            # Compute Constraints for The Path
            for waypoint_index in range(len(PWL)):
                P_star = PWLs_in[idx_a][waypoint_index]
                P      = PWL[waypoint_index]

                m.addConstrs(
                    P_star[0][i] == P[0][i] for i in range(dims)
                )

        if tasks is not None:
            for idx_agent in range(len(tasks)):
                size_a = size_list[idx_agent]
                for idx_task in range(len(tasks[idx_agent])):
                    self.handleSpecTree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size_a)
                    self.handleSpecTree(
                        copy.deepcopy(tasks[idx_agent][idx_task]), PWLs_z3[idx_agent], bloat, size_a
                    )

            conjunctions = []
            for idx_task in range(len(tasks[0])):
                disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                conjunctions.append(self.Disjunction(disjunctions))
            z = self.Conjunction(conjunctions)
            self.add_CDTree_Constraints(m, s, z, copy.deepcopy(z))

        if not ignore_clearance_constraints:
            add_mutual_clearance_constraints(m, PWLs, bloat)

        # obj = sum([L1Norm(m, _sub(PWL[i][0], PWL[i+1][0])) for PWL in PWLs for i in range(len(PWL)-1)])
        obj = sum([PWL[-1][1] for PWL in PWLs])
        m.setObjective(obj, GRB.MINIMIZE)

        m.write("plan-check.lp")
        print('NumBinVars: %d'%m.getAttr('NumBinVars'))

        start = time.time()
        m.optimize()
        end = time.time()
        print('solving it takes %.3f s'%(end - start))

        return end-start, m.getAttr('Status')

    """
    add_binary_assignment_constraints
    Description:
        Checks whether or not a given plan satisfies the specification given.
    """
    def add_binary_assignment_constraints(self, x0s, specs, bloat, num_segs, limits=None, tasks=None, vmax=3., MIPGap=1e-4, max_segs=None, tmax=None, hard_goals=None, size_list=[],ignore_clearance_constraints=False):
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
        for spec in specs:
            self.clearSpecTree(spec)
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

            PWL_z3 = self.copyGurobiPWLToZ3(PWL)
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
            #add_velocity_constraints_smt(s, PWL_z3, vmax=vmax)

            add_time_constraints(m, PWL, tmax)
            #add_time_constraints_smt(s, PWL_z3, tmax)

            # Get Size of the object
            size_a = size_list[idx_a]
            self.handleSpecTree(spec,    PWL,   bloat, size_a)
            self.handleSpecTree(spec_z3, PWL_z3,bloat, size_a)
            self.add_CDTree_Constraints(m, s, spec.zs[0] , spec_z3.zs[0] )

        if tasks is not None:
            for idx_agent in range(len(tasks)):
                size_a = size_list[idx_agent]
                for idx_task in range(len(tasks[idx_agent])):
                    self.handleSpecTree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size_a)
                    self.handleSpecTree(
                        copy.deepcopy(tasks[idx_agent][idx_task]), PWLs_z3[idx_agent], bloat, size_a
                    )

            conjunctions = []
            for idx_task in range(len(tasks[0])):
                disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                conjunctions.append(self.Disjunction(disjunctions))
            z = self.Conjunction(conjunctions)
            self.add_CDTree_Constraints(m, s, z, copy.deepcopy(z))

        if not ignore_clearance_constraints:
            add_mutual_clearance_constraints(m, PWLs, bloat)

        self.smt_solver = s

        return

    """
    mu
    Description:
        A custom built mu that uses our own Conjunction object.
    """
    def mu(self, i, PWL, bloat_factor, A, b):
        bloat_factor = np.max([0, bloat_factor])
        # this segment is fully contained in Ax<=b (shrinked)
        b = b.reshape(-1)
        num_edges = len(b)
        conjunctions = []
        conjunctions_z3 = []
        for e in range(num_edges):
            a = A[e,:]
            for j in [i, i+1]:
                x = PWL[j][0]
                # x_z3 = PWL_z3[j][0]
                conjunctions.append(b[e] - np.linalg.norm(a) * bloat_factor - sum([a[k]*x[k] for k in range(len(x))]) - EPS)
                # conjunctions_z3.append(b[e] - np.linalg.norm(a) * bloat_factor - sum([a[k]*x_z3[k] for k in range(len(x_z3))]) - EPS)
        
        return self.Conjunction(conjunctions)

    """
    negmu
    Description:
        A custom built negmu that uses our own Conjunction and Disjunction node objects.
    """
    def negmu(self, i, PWL, bloat_factor, A, b):
        # this segment is outside Ax<=b (bloated)
        b = b.reshape(-1)
        num_edges = len(b)
        disjunctions = []
        for e in range(num_edges):
            a = A[e,:]
            conjunctions = []
            # conjunctions_z3 = []
            for j in [i, i+1]:
                x = PWL[j][0]
                # x_z3 = PWL_z3[j][0]
                conjunctions.append(sum([a[k]*x[k] for k in range(len(x))]) - (b[e] + np.linalg.norm(a) * bloat_factor) - EPS)
                #conjunctions_z3.append(sum([a[k]*x_z3[k] for k in range(len(x))]) - (b[e] + np.linalg.norm(a) * bloat_factor) - EPS)
            disjunctions.append(self.Conjunction(conjunctions))
        return self.Disjunction(disjunctions)


    def always(self, i, a, b, zphis, PWL):
        t_i = PWL[i][1]
        t_i_1 = PWL[i+1][1]
        conjunctions = []
        for j in range(len(PWL)-1):
            t_j = PWL[j][1]
            t_j_1 = PWL[j+1][1]
            # print(t_j, t_j_1)
            # print(t_i+a)
            # print(t_i_1)
            # print(b)
            # print(t_i_1 + b)
            # print(zphis[j])
            conjunctions.append(self.Disjunction([self.noIntersection(t_j, t_j_1, t_i + a, t_i_1 + b), zphis[j]]))
        return self.Conjunction(conjunctions)

    def noIntersection(self, a, b, c, d):
        # z = 1 iff. [a, b] and [c, d] has no intersection
        # b < c or d < a
        return self.Disjunction([c-b-EPS, a-d-EPS])

    def hasIntersection(self, a, b, c, d):
        # z = 1 iff. [a, b] and [c, d] has no intersection
        # b >= c and d >= a
        return self.Conjunction([b-c, d-a])

    def eventually(self, i, a, b, zphis, PWL):
        t_i = PWL[i][1]
        t_i_1 = PWL[i+1][1]
        z_intervalWidth = b-a-(t_i_1-t_i)-EPS
        disjunctions = []
        for j in range(len(PWL)-1):
            t_j = PWL[j][1]
            t_j_1 = PWL[j+1][1]
            disjunctions.append(self.Conjunction([self.hasIntersection(t_j, t_j_1, t_i_1 + a, t_i + b), zphis[j]]))
        return self.Conjunction([z_intervalWidth, self.Disjunction(disjunctions)])