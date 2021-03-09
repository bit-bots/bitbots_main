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

First we need to have the current simplified URDF model. To get this look at the documentation of the wolfang_description package :doc:`../wolfgang_description/index`.

To use the URDF in webots we need to create a .proto file from the URDF. To do this clone the urdf2robot repository

.. code-block:: bash

  git clone git@github.com:cyberbotics/urdf2webots.git
  cd urdf2webots

Run the script to adapt the urdf to be usable by webots2urdf

.. code-block:: bash

  roscd wolfgang_webots_sim
  python scripts/fix_urdf_for_webots.py ../wolfgang_description/urdf/robot.urdf webots_robot.urdf

Run the conversion script from urdf to proto file

.. code-block:: bash

  python urdf2webots/demo.py --input webots_robot.urdf --output protos  --multi-file --box-collision --link-to-def

Unfortunately, there are four things that need to be done manually.
Firstly, add the line

`field SFFloat cameraFOV 1.04`

to the header of the proto.

Secondly, replace the number behind `fieldOfView` in the `camera_optical_frame` with `IS cameraFOV`.

Thirdly, delete the compass from both IMUs.

Fourthly, add the following fields into both imu `Solid` nodes. Afterwards the Solids should have the following fields: translation, rotation, physics, boundingObject, children(Accelerometer, Gyo), name

.. code-block:: json

    physics Physics {
    }
    boundingObject Box {
    size 0.01 0.01 0.01
    }

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
