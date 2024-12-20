# Setting up runtime type checking for this package
# We need to do this again here because the dsd imports
# the decisions and actions from this package in a standalone way
from beartype.claw import beartype_this_package

beartype_this_package()
