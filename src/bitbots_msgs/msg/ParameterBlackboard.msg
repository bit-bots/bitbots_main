# Complete state of the global parameter blackboard (e.g. game settings and field dimensions).
# The parameter_blackboard node publishes it latched (transient local) on startup and every time a parameter changes,
# so late subscribers always receive the current state.

# Time at which this state was published
builtin_interfaces/Time stamp

# All parameters that are currently set on the blackboard
rcl_interfaces/Parameter[] parameters
