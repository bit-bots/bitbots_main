Welcome to |project|'s documentation!
================================================

Description
-----------

This is the vision ROS package of the Hamburg Bit-Bots. A description of the current, YOEO-based vision can be found
here: :doc:`manual/yoeo_vision`. For the old legacy vision, look here: :doc:`manual/legacy_vision`.

Launchscripts
-------------

To start the vision, use

::

   ros2 launch bitbots_vision vision.launch

The following parameters are available:

+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|Param                |Default  |Output                                                                                                                     |
+=====================+=========+===========================================================================================================================+
|``sim``              |``false``|Activate simulation time, switch to simulation color settings and deactivate launching of an image provider                |
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+



.. toctree::
   :maxdepth: 2
   :caption: Interface documentation

   cppapi/library_root
   pyapi/modules

.. toctree::
    :maxdepth: 1
    :glob:
    :caption: Manuals

    manual/*

.. toctree::
    :maxdepth: 1
    :glob:
    :caption: Tutorials

    manual/tutorials/*

Indices and tables
==================

* :ref:`genindex`
* |modindex|
* :ref:`search`
