"""
intrateam_plan.py
Description:
    This file should contain a few of the files necessary for intrateam planning.
"""

import numpy as np
from gurobipy import *

"""
intrateam_plan
Description:
    Plan for just the team members given the team trajectory.
    Team members should:
    - Not collide with one another
    - Remain within an ellipsoid of the size given in team_size    
"""
def intrateam_plan(team_traj:List{Tuple}, x0s:List{np.array},n_members:int,team_radius:float):
    # Constants

    # Create Trajectories for each of the members.
    PWLs = []
    m = Model("intrateam_planning - " + str(n_members) + " members")
    # m.setParam(GRB.Param.OutputFlag, 0)
    m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
    m.setParam(GRB.Param.MIPGap, MIPGap)
    # m.setParam(GRB.Param.NonConvex, 2)
    # m.getEnv().set(GRB_IntParam_OutputFlag, 0)

    for idx_a in range(len(x0s)):
        x0 = x0s[idx_a]

    # Create constraints based on the team_traj
    