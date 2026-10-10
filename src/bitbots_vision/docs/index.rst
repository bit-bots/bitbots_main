Welcome to |project|'s documentation!
================================================

Description
-----------

This is the vision ROS package of the Hamburg Bit-Bots. A detailed description of the current, RF-DETR-based vision
can be found here: :doc:`manual/rfdetr_vision`.

Launchscripts
-------------

To start the vision, use

::

   ros2 launch bitbots_vision vision.launch

The following arguments are available:

+---------------------+---------+-------------------------------------------------+
|Arg                  |Default  |Description                                      |
+=====================+=========+=================================================+
|``sim``              |``false``|Activate simulation time                         |
+---------------------+---------+-------------------------------------------------+
|``debug``            |``false``|Activate publishing of the debug image           |
+---------------------+---------+-------------------------------------------------+



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
