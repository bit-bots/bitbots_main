# bitbots_meta
This git contains all RoboCup code from the Hamburg Bit-Bots as submodules as well as documentation.
All code is written as ROS packages and tested for ROS-Melodic.


Structure
---------

The actual code is in submodule gits to make it easier for others to use only parts of our code.
If you want to pull all submodule run

``` bash
make pull-all
```
The naming prefix indicates the scope of the packages.

 * bitbots_ : specific RoboCup code of our team which follows interface specification of humanoid_league_msgs
 * humanoid_league_ : packages which are useful for all teams in the RoboCup Humanoid League, e.g. visualization tools and gamecontroller
 * no prefix : packages which are useful in general and usable outside of RoboCup


Documentation
-------------

Our documentation is WIP but what is already available is hosted on [doku.bit-bots.de](http://doku.bit-bots.de/meta/)

You can also build the documentation yourself
``` bash
make doc
```

and open it with

``` bash
firefox doc/html/index.html
```

You can also generate an overview of the software using the script in the architecture folder.

