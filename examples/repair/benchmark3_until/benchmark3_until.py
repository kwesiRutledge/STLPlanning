"""
benchmark3-until.py
Description:
    Creates code for the third benchmark problem.
    It involves a single agent moving and trying to satisfy an 'until' specification.
"""

import numpy as np
import sys
sys.path.append('../../../')
#from PWLPlan import plan2
from src.plot_plan import plot_plan


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
        (np.array([1.0,2.0]),0.0),
        (np.array([1.25,2.0]),2.0),
        (np.array([1.5,2.0]),4.0),
        (np.array([1.75,2.0]),6.0),
        (np.array([2.0,2.0]),7.0),
        (np.array([2.25,2.0]),8.0),
        (np.array([2.5,2.0]),10.0),
        (np.array([2.75,2.0]),12.0),
        (np.array([3.0,2.0]),14.0),
        (np.array([3.25,2.0]),16.0),
        (np.array([3.25,2.5]),18.0)
    ] # Example Curve

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

    b1 = np.array([-3.0, 3.5, -2.25, 2.75], dtype = np.float64)
    goals = [(A, b1)]

    plots = [[goals, 'g'], [obs, 'k']]

    return pwl_curve, plots

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
    b = np.array([-1.9, 3.0, -2.25, 2.75], dtype = np.float64)
    plot_tuples1.append(
        [ [(A,b)] , 'm' ]
    )

    return plot_tuples1

if __name__ == '__main__':
    # Create Initial Context
    pwl_curve0, plots0 = create_optimistic_context()

    fig1 = plot_plan([pwl_curve0],plots0)
    fig1.savefig("benchmark3-until_clear.png")

    plots1 = create_real_context(plots0)

    fig2 = plot_plan([pwl_curve0],plots1)
    fig2.savefig("benchmark3-until_desirable.png")
