std_msgs/Header header
builtin_interfaces/Time t0  # ROS time of the initial state (odom stamp) the tube starts from
float64 dt  # step k = 1..N of the tube is predicted at t0 + k*dt
uint8 id  # SystemID of the sending agent (0..255)
# Tube of N stars, each advanced from the initial state, rebuilt with build_probstar(mean[k], std[k]).
# Flattened step by step: step k is [6(k-1), 6k), x, y, z, vx, vy, vz (earth frame)
float64[] mean
float64[] std
# Bound on |true - linearized| state of each step (linearization_error), same layout as mean
float64[] err
