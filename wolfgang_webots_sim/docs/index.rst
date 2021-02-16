Welcome to |project|'s documentation!
================================================

Description
-----------

To generate the webots proto file from a URDF, we use urdf2webots_.

Unfortunately, this script does not handle links which contain an inertial element but no collision or visual element correctly.
Pybullet on the other hand requires an inertial component for each link.
To remove these inertial components for webots2urdf there is a script which is later explained.

.. _urdf2webots: https://github.com/cyberbotics/urdf2webots

Instructions
------------

These instructions assume that you are in the wolfgang_webots_sim package.
To get there use

.. code-block:: bash

  roscd wolfgang_webots_sim

Firstly clone the urdf2robot repository

.. code-block:: bash

  git clone git@github.com:cyberbotics/urdf2webots.git
  cd urdf2webots

Run the script to adapt the urdf to be usable by webots2urdf

.. code-block:: bash

  roscd wolfgang_webots_sim
  python scripts/fix_urdf_for_webots.py ../wolfgang_description/urdf/robot.urdf webots_robot.urdf

Run the conversion script from urdf to proto file

.. code-block:: bash

  python urdf2webots/demo.py --input webots_robot.urdf --output protos  --multi-file --box-collision --disable-mesh-optimization

After verifying that the proto file is loaded correctly, you can enable the mesh optimization.

In the future the box collision will be replaced by simplified models of the links.

Currently the camera and IMUs are not placed in the correct frame. This will be fixed with a PR that is currently open.


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
