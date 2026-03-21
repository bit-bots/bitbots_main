# This represents a point in free space with uncertainty.
geometry_msgs/Point point

# Row-major representation of the 3x3 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z)
float64[9] covariance
