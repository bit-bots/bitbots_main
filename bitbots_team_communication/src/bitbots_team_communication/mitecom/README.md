
Mixed Team Communication protocol
=================================

TODO: Introduction


Idea
----

TODO


Roadmap
-------

TODO


Design Principles
-----------------

TODO


Running the example
-------------------

To compile the example, check out the project and type `make`. Next, start the created
`mitecom` binary. It requires 4 parameters: the local port to use, the port to broadcast
to, the ID of the robot and the ID of the team. You need to start at least two instances.

If you have multiple computers, just run `./mitecom 12321 12321 X 1` (replace X with
individual robot IDs on each computer). If you only have one computer, you can swap
ports by running

    ./mitecom 12321 12345 1 1

in one terminal and

    ./mitecom 12345 12321 2 1

in another.


Instructions
------------

TODO


File Overview
-------------

The following files are provided:

    mitecom-data.h         Contains the definition of data structures and enums
    mitecom-handler.*      De/serializing functions

    mitecom-network.*      Example network handling code
    mitecom-roledecider.*  Example (common) logic to decide a role
    mitecom-main.cpp       Example main function demonstrating the functionality

To adapt the protocol, you need at least mitecom-data.h and mitecom-handler.*.
The other functionality may already be available in your framework, alternatively
you can use the logic from the example code (network, main).


Related Works
-------------

TODO


Feedback
--------

We welcome contributions to this project.


Acknowledgments
---------------

This project was sponsored under a project grant of the RoboCup Federation.

