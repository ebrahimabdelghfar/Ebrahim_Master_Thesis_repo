# Short-window peak-friction fit from On-Track-SysID's brush estimator, published
# far more often than the Pacejka coefficient set reaches mpc/update_params.
#
# This is an OPTIONAL derate. A consumer that never receives it must fall back to
# the peak friction implied by the identified Pacejka D, so nothing is published
# at all when the estimator is disabled or never passes its release gate.
std_msgs/Header header

# Peak friction of the front axle, dimensionless. The front is the only axle this
# vehicle excites enough to identify; it speaks for both on one road surface.
float64 mu

# One standard deviation of `mu` from the same fit. A short window buys latency at
# the cost of confidence, so a consumer tightens its grip margin as this grows.
float64 sigma_mu

# False when the fit did not pass the release gate this tick, which is normal on a
# straight. `mu` and `sigma_mu` are then meaningless and must be ignored.
bool valid
