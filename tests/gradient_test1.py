"""
gradient_test1.py
Description:
    Testing the use of jax's grad feature on our stl robustness metric stuff.
"""
import jax.numpy as jnp
from jax import grad
import sys

sys.path.append("../")
from src.robustness.smooth_operators import rho_mu_smooth1, rho_negmu_smooth1


# Creating a curve

pwl_curve0 = [
    (jnp.array([0.0,0.0]),0.0),
    (jnp.array([0.25,0.25]),2.0),
    (jnp.array([0.5,0.5]),4.0),
    (jnp.array([0.5,0.7]),6.0),
    (jnp.array([0.6,0.85]),7.0),
    (jnp.array([0.7,1.0]),8.0),
    (jnp.array([1.0,1.3]),10.0)
] # Example Curve

obstacle0 = (
    jnp.vstack((jnp.eye(2),-jnp.eye(2))),
    jnp.array([1.0,1.0,-0.5,1.0])
) # Obstacle

print(obstacle0[0])
print(obstacle0[1])

# Evaluate trajectory with respect to this obstacle using rho_mu_smooth1
print("Robustness value when the trajectory is not touching the target region at all (should not satisfy spec, rho < 0):")
print("rho_mu_smooth1(0,pwl_curve0,obstacle0[0],obstacle0[1]) =",rho_mu_smooth1(0,pwl_curve0,obstacle0[0],obstacle0[1]) )
print("Robustness value of trajectory when it is in the target region at all (should satisfy spec, rho > 0):")
print("rho_mu_smooth1(4,pwl_curve0,obstacle0[0],obstacle0[1]) =",rho_mu_smooth1(4,pwl_curve0,obstacle0[0],obstacle0[1]) )

print(" ")
print("Testing negmu:")
print("Robustness value when the trajectory is not touching the target region at all (should satisfy spec, rho > 0):")
print("rho_negmu_smooth1(0,pwl_curve0,obstacle0[0],obstacle0[1]) =",rho_negmu_smooth1(0,pwl_curve0,obstacle0[0],obstacle0[1]) )
print("Robustness value of trajectory when it is in the target region at all (should NOT satisfy spec, rho < 0):")
print("rho_negmu_smooth1(4,pwl_curve0,obstacle0[0],obstacle0[1]) =",rho_negmu_smooth1(4,pwl_curve0,obstacle0[0],obstacle0[1]) )

# Creating Function of JUST the trajectory for a given obstacle.
test_rho1 = lambda pwl_curve : rho_mu_smooth1(4,pwl_curve,obstacle0[0],obstacle0[1])

grad_rho1 = grad(test_rho1)
print("Taking a gradient!")
print(grad_rho1(pwl_curve0))

# Plot the gradient on a figure with the obstacles and everything else
import matplotlib.pyplot as plt
import numpy as np

test_rho2 = lambda pwl_curve_in : rho_negmu_smooth1(4,pwl_curve_in,obstacle0[0],obstacle0[1])
grad_rho2 = grad(test_rho2)
grad2 = grad_rho2(pwl_curve0)
nonzero_grads = [ grad2[ts_index][0] for ts_index in range(3,5+1) ]

constant_axis = [-0.1,1.1,-0.1,1.5]

# Plot obstacle and trajectory alone
plt.figure()
plt.plot()
# Plot the box!
plt.plot(
    [0.5,1.0,1.0,0.5,0.5],
    [1.0,1.0,-1.0,-1.0,1.0],
    color="magenta"
)
# Plot all trajectory points
plt.plot(
    [timed_waypoint[0][0] for timed_waypoint in pwl_curve0],
    [timed_waypoint[0][1] for timed_waypoint in pwl_curve0]
)

plt.axis(constant_axis)

plt.savefig("../images/gradient_test1-fig1-empty.png")

plt.figure()
plt.plot()
# Plot the box!
plt.plot(
    [0.5,1.0,1.0,0.5,0.5],
    [1.0,1.0,-1.0,-1.0,1.0],
    color="magenta"
)

# Plot all trajectory points
plt.plot(
    [timed_waypoint[0][0] for timed_waypoint in pwl_curve0],
    [timed_waypoint[0][1] for timed_waypoint in pwl_curve0]
)

for temp_grad_index in range(len(nonzero_grads)):
    grad_at_timeseg = nonzero_grads[temp_grad_index]
    print(grad_at_timeseg)
    x_at_timeseg = pwl_curve0[3+temp_grad_index][0]
    print(x_at_timeseg)
    plt.arrow(
        x_at_timeseg[0],x_at_timeseg[1],
        0.1*grad_at_timeseg[0],0.1*grad_at_timeseg[1],
        width=0.01
    )
plt.axis(constant_axis)

plt.savefig("../images/gradient_test1-fig1-with-grads.png")

