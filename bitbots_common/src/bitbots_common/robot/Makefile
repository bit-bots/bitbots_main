# This Makefile is in some kind a link to the Makefile in the project root.
# When invoked with no target is performs a make vision with the root Makefile
# Otherwise it's a target forward.

local:
	cd .. ; make pypose robot kinematics

%:
	cd .. ; make $@
