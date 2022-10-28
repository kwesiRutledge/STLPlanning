"""
smooth_operators.py
Description:
    This file contains the smooth operators which approximately compute robustness over a signal (defined as timed waypoints).
    The idea comes from 'A Smooth Robustness Measure of Signal Temporal Logic for Symbolic Control' by Vince Kurtz.
"""

import numpy as np
import jax.numpy as jnp

"""
find_x_at_t(t_interest,pwl_curve)
Description:
    Retrieves the value of the state at time t using the sequence of timed-waypoints.
    Assumes that t is in the range defined by pwl_curve
"""
def find_x_at_t( t_interest:float , pwl_curve ):
    # Constants
    times = [ t_waypoint[1] for t_waypoint in pwl_curve ]

    # Error Handling
    max_t = max(times)
    if t_interest > max_t:
        raise(Exception("The time you are interested in (" + str(t_interest) + ") is outside of the trajectory provided by pwl_curve (maximum t=" + str(max_t) + ")."))

    # If the time of interest is EXACTLY one of the waypoints, then return it.
    if t_interest in times:
        print("Time is one of the waypoints! Returning it based on its index!")
        for t_waypoint in pwl_curve:
            if t_waypoint[1] == t_interest:
                return t_waypoint[0]
    
    # If the time of interest is not exactly one of the waypoints, then we need to interpolate.
    t_lb_index = (np.array(times) > t_interest).index(True) - 1
    t_ub_index = (np.array(times) < t_interest).index(True) + 1 # Shouldn't these be one away from each other always?

    x_lb, t_lb = pwl_curve[t_lb_index]
    x_ub, t_ub = pwl_curve[t_ub_index]

    return (t_interest - t_lb)/(t_ub - t_lb) * x_lb + (t_ub - t_interest)/(t_ub - t_lb) * x_ub


"""
smooth_min
Description:
    Smooth form of the minimum function.
    Uses jax expressions.
"""
def smooth_min(args_in:jnp.array):
    # Constants
    k1 = 10

    # Compute Smooth min
    temp_sum = jnp.sum(jnp.exp(-k1*args_in)) # Vectorized exponential call and then summed the result.
    return -(1/k1)*jnp.log(temp_sum)

"""
smooth_max
Description:
    Smooth form of the maximum function.
    Uses jax expressions.
"""
def smooth_max(args_in:jnp.array):
    # Constants
    k2 = 10

    # Compute Smooth min
    temp_sum = jnp.sum(jnp.exp(k2*args_in)) # Vectorized exponential call and then summed the result.
    return (1/k2)*jnp.log(temp_sum)


"""
rho_mu_smooth1
Description:
    Computes the SMOOTHED robustness value (rho) for the atomic proposition mu which is
    represented by the polytope defined by A x <= b in the state space.
"""
def rho_mu_smooth1(segment_index:int,pwl_curve,A:jnp.asmatrix,b:jnp.array):
    # Constants
    # n_hyperpl = len(b)

    # Evaluate the distance to each hyperplane described by the polytope (A x <= b)
    x_i = pwl_curve[segment_index][0]
    x_ip1 = pwl_curve[segment_index+1][0]

    # We will reinterpret the meaning of this formula to instead be for the SEGMENT
    return smooth_min(
        jnp.array( jnp.vstack( (b - A.dot(x_i) , b - A.dot(x_ip1) )) )
    )

"""
rho_negmu_smooth1
Description:
    Computes the SMOOTHED robustness value (rho) for the atomic proposition mu which is
    represented by the polytope defined by A x <= b in the state space.
"""
def rho_negmu_smooth1(segment_index:int,pwl_curve,A:jnp.asmatrix,b:jnp.array):
    # Constants
    # n_hyperpl = len(b)

    # Evaluate the distance to each hyperplane described by the polytope (A x <= b)
    x_i = pwl_curve[segment_index][0]
    x_ip1 = pwl_curve[segment_index+1][0]

    # We will reinterpret the meaning of this formula to instead be for the SEGMENT
    minimums = []
    for hyperplane_index in range(len(b)):
        A_i = A[hyperplane_index,:]
        b_i = b[hyperplane_index]

        minimums.append(
            smooth_min( jnp.array([A_i.dot(x_i) - b_i,A_i.dot(x_ip1) - b_i]) )
        )

    return smooth_max(
        jnp.array( minimums )
    )
    