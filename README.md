# bitbots_meta
This git contains all RoboCup code from the Hamburg Bit-Bots as submodules as well as documentation.
All code is written as ROS packages and tested for Kinetic and Melodic.


Structure
---------

The actual code is in submodul gits to make it easier for other to use only parts of our code.
If you want to pull all submodule run

.. code::bash
    make pull-all

The naming prefix indicates the scope of the packages.

 * bitbots_ : specific RoboCup code of our team which follows interface specification of humanoid_league_msgs
 * humanoid_leage_ : packages which are useful for all teams in the RoboCup Humanoid League, e.g. visualization tools and gamecontroller
 * no prefix : packages which are useful in general and usable outside of RoboCup




Documentation
-------------

To build the documentation do the following
.. code:: bash
    make doc

Open it with

.. code:: bash
    firefox doc/html/index.html

You can also generate an overview of the software using the script in the architecture folder.