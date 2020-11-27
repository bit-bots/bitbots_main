Welcome to |project|'s documentation!
================================================

Description
-----------

To generate the webots proto file from a URDF, we use urdf2webots_.

Unfortunately, this script does not handle links with a inertial component
but without a collision or visual model correctly.
Pybullet requires an inertial component for each link.

.. _urdf2webots: https://github.com/cyberbotics/urdf2webots

Instructions
------------

These instructions assume that you are in the wolfgang_webots_sim package.
To get there use

.. code-block:: bash
  roscd wolfgang_webots_sim


Firstly clone the urdf2robot repository

.. code-block:: bash

  git clone git@github.com:jgueldenstein/urdf2webots.git
  cd urdf2webots
  git checkout "fix/rotated_bounding_objects"

Run the script to adapt the urdf to be usable by webots2urdf

.. code-block:: bash

  roscd wolfgang_webots_sim
  python scripts/fix_urdf_for_webots.py ../wolfgang_description/urdf/robot.urdf webots_robot.urdf

Run the converstion script from urdf to proto file

.. code-block:: bash

  python urdf2webots/demo.py --input webots_robot.urdf --output protos --rotation="1 0 0 -1.5708" --multi-file --box-collision --disable-mesh-optimization

After verifying that the proto file is loaded correctly, you can enable the mesh optimization.