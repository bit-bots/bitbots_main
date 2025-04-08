Welcome to |project|'s documentation!
================================================

Description
-----------
This robot model is exported from onshape by using onshape-to-robot.
See their `documentation <https://onshape-to-robot.readthedocs.io/en/latest/>`_ on how this works.

We do not want to use the exact model as a collision model, since this would make the simulation run slowly.
Therefore, we first need to simplify the model by using the following.

.. code-block:: bash

    roscd wolfgang_description
    cd urdf
    rosrun simplify_urdf_collision simplify.py robot.urdf robot.urdf -r -s

The script will ask you which links should be excluded for the simplification. Normally, you don't need to exclude any links, since the important collision models (arms or feet) already have manually created collision models which are not shown in this list.

|description|

.. toctree::
   :maxdepth: 2

   cppapi/library_root
   pyapi/modules


Indices and tables
==================

* :ref:`genindex`
* |modindex|
* :ref:`search`
