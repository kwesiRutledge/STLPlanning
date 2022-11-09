"""
intrateam_plan.py
Description:
    This file should contain a few of the files necessary for intrateam planning.
"""

import numpy as np
from gurobipy import *

import sys, time
sys.path.append("../")
from PWLPlan import IntFeasTol, add_mutual_clearance_constraints, noIntersection, Conjunction, Disjunction, add_CDTree_Constraints

"""
disjoint_segments2
Description:
    This function rips out the original logic from add_mutual_clearance_constraints
    and attempts to write them in terms of a 
"""
def disjoint_segments2(model, seg1, seg2, bloat):
    assert(len(seg1) == 2)
    assert(len(seg2) == 2)
    # assuming that bloat is the error bound in two norm for one agent

    # Create a Mixed-Integer Condition
    disjunctions = []
    for endpoint_index in range(1,len(seg1)):
        #For each endpoint, 
        endpoint_i = seg1[endpoint_index]
        partner_endpoint = seg1[1-endpoint_index]
        
        # create a constraint such that:
        # - all points are above 
        # - and the partner is below
        # a bloated hyperplane
        for dim_index in range(1,len(endpoint_i)):
            conjunctions = []
            conjunctions.append( seg2[0][dim_index] - (endpoint_i[dim_index] + bloat)  ) # endpoint_i[dim_index] + bloat <= seg2[0][dim_index]
            conjunctions.append( seg2[1][dim_index] - (endpoint_i[dim_index] + bloat) ) # endpoint_i[dim_index] + bloat <= seg2[1][dim_index]
            
            conjunctions.append( endpoint_i[dim_index] - partner_endpoint[dim_index]  ) # endpoint_i[dim_index] >= partner_endpoint[dim_index]
            
            # Add this to the list of disjunctions
            disjunctions.append(
                Conjunction(conjunctions)
            )


        # or the constraint that:
        # - all seg2 points are below this one in terms of the simple hyperplanes
        # - and the partner is above
        for dim_index in range(1,len(endpoint_i)):
            conjunctions = []
            conjunctions.append( (endpoint_i[dim_index] - bloat) - seg2[0][dim_index] ) # endpoint_i[dim_index] + bloat >= seg2[0][dim_index]
            conjunctions.append( (endpoint_i[dim_index] - bloat) - seg2[1][dim_index] ) # endpoint_i[dim_index] + bloat >= seg2[1][dim_index]
            
            conjunctions.append( partner_endpoint[dim_index] - endpoint_i[dim_index]   ) # endpoint_i[dim_index] <= partner_endpoint[dim_index]
            
            # Add this to the list of disjunctions
            disjunctions.append(
                Conjunction(conjunctions)
            )


    return Disjunction(disjunctions)


"""
add_mutual_clearance_constraints2(m, PWLs, bloat)
Description:
    Slightly tweaking (really I'm aggressively tweaking) the clearance function to be a bit less conservative.
"""
def add_mutual_clearance_constraints2(model, PWLs, bloat):
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
                    z_disjoint_segments = disjoint_segments2(model, [x11, x12], [x21, x22], bloat)
                    z = Disjunction([z_noIntersection, z_disjoint_segments])
                    add_CDTree_Constraints(model, z)

"""
intrateam_plan(team_traj:List{Tuple}, x0s:List{np.array},n_members:int,team_radius:float,MIPGap=1e-4)
Description:
    Plan for just the team members given the team trajectory.
    Team members should:
    - Not collide with one another
    - Remain within an ellipsoid of the size given in team_size    
"""
def intrateam_plan( team_traj:list[tuple] , x0s:list[np.array] , team_radius:float , bloat:float = 0.0, agent_radius_list=[], MIPGap=1e-4 , base_model_name: str="intrateam_planning" , use_original_clearance_constraint: bool=True ):
    # Constants
    n_members = len(x0s)
    num_segs = len(team_traj)-1
    dims = len(team_traj[0][0])

    bloat = 0.0

    # Input Checking?

    # Default Values Setup
    default_radius = 0.11*4/2
    if len(agent_radius_list) == 0:
        agent_radius_list = [default_radius for i in range(n_members) ]

    # Create Trajectories for each of the members.
    PWLs = []
    m = Model( base_model_name + " - " + str(n_members) + " members")
    # m.setParam(GRB.Param.OutputFlag, 0)
    m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
    m.setParam(GRB.Param.MIPGap, MIPGap)
    # m.setParam(GRB.Param.NonConvex, 2)
    # m.getEnv().set(GRB_IntParam_OutputFlag, 0)

    # Algorithm 
    for idx_a in range(len(x0s)):
        x0 = x0s[idx_a]
        radius_a = agent_radius_list[idx_a]

        # Create variables for each curve
        PWL = []
        for i in range(num_segs+1):
            PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
        PWLs.append(PWL)
        m.update()

        # Add constraint on initial conditions of each team member
        for dim_index in range(dims):
            m.addConstr( PWL[0][0][dim_index] == x0[dim_index] )

        # Add constraints that the states stay close to the given values
        # - times stay close
        for endpt_index in range(num_segs+1):
            m.addConstr(PWL[endpt_index][1] == team_traj[endpt_index][1])

        # - states stay close (l_2 distance)
        for endpt_index in range(num_segs+1):
            x_a_t = MVar(PWL[endpt_index][0])
            x_target_t = team_traj[endpt_index][0]
            m.addConstr(
                ( x_a_t[0] - x_target_t[0] )*( x_a_t[0] - x_target_t[0] ) + ( x_a_t[1] - x_target_t[1] )*( x_a_t[1] - x_target_t[1] ) <= np.power(team_radius-radius_a,2)
            )

    if use_original_clearance_constraint:
        add_mutual_clearance_constraints(m, PWLs, bloat)
    else:
        add_mutual_clearance_constraints2(m,PWLs, bloat)
    
    # Write the Model into a file
    m.write( base_model_name + ".lp")

    optimization_time = -1.0
    PWLs_output = []

    # Solve
    try:
        print('NumBinVars: %d'%m.getAttr('NumBinVars'))

        start = time.time()
        m.optimize()
        end = time.time()
        print('solving it takes %.3f s'%(end - start))
        optimization_time = end - start

        
        for PWL in PWLs:
            PWL_output = []
            for P in PWL:
                PWL_output.append(
                    ( np.array([P[0][i].X for i in range(len(P[0]))]), P[1].X)
                    )
            PWLs_output.append(PWL_output)

        m.dispose()

    except Exception as e:
        print("Exception occurred! ",e)

        # Dispose of Model
        m.dispose()

    # Create outputs
    return PWLs_output, optimization_time
    