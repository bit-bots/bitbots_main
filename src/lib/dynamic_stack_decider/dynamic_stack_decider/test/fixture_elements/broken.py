"""A module that fails to import. Discovery must skip it and log the error, not crash."""

raise RuntimeError("this fixture module cannot be imported")
