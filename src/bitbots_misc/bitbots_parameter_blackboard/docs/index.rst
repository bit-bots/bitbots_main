Welcome to |project|'s documentation!
================================================

Description
-----------

|description|

The ``parameter_blackboard`` node loads the global configuration itself.
It merges the configuration of the selected field (``config/fields/<field.name>/config.yaml``),
``config/global_parameters.yaml``, ``config/game_settings.yaml`` and, in simulation, the game settings of the
simulated robot that matches ``ROS_DOMAIN_ID``.
Later files take precedence, parameters passed to the node directly (e.g. by the launch file) take precedence over all
files.
The launch file accepts a ``parameters_file`` with such overrides, which ``bitbots_orchestrate`` uses to pass
``--global`` values on.
Overrides may only replace parameters that the files define, with a value of the same type;
otherwise the node refuses to start, so that a typo does not silently add a parameter.
See ``parameter_files`` in ``bitbots_parameter_blackboard/parameter_blackboard.py`` for the exact order.

The configuration is exposed in two ways:

* The standard ROS parameter services of the node, e.g. ``ros2 param get /parameter_blackboard bot_id``.
  Parameters can also be set or added at runtime, except for ``field.name``, as the field maps depend on it.
* The latched (transient local) ``global_parameter_blackboard`` topic of type ``bitbots_msgs/msg/ParameterBlackboard``.
  It contains the complete state of the blackboard and is republished on every parameter change.
  This is the preferred way for nodes to get global parameters, use ``get_global_parameters`` from
  ``bitbots_utils`` (available for Python and C++), which waits for the blackboard and reports missing parameters.

Additionally, the node publishes the maps of the selected field as latched ``nav_msgs/msg/OccupancyGrid`` messages
on ``field/map/<name>`` (e.g. ``field/map/lines``), if the field provides them.
See ``FIELD_MAPS`` in ``bitbots_parameter_blackboard/parameter_blackboard.py`` for the available maps.
They are used by the localization and can be visualized in RViz.

.. toctree::
   :maxdepth: 2

   cppapi/library_root
   pyapi/modules


Indices and tables
==================

* :ref:`genindex`
* |modindex|
* :ref:`search`
