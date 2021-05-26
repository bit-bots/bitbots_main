Executing tests
===============

There are multiple ways to execute tests.
Generally you can either run them via the catkin build system or for python based test via an external test runner.
The difference between them is mostly that they differ in the amount of information they provide when a test fails.


Execute all Tests via Catkin
----------------------------

This is the default way to simply execute all tests for a project and is also how *Jenkins* will run them.

:Simple Version:

   ``catkin test <package>`` is a *catkin* alias which will run all tests for the given package as well as all
   dependencies.

:Without Dependencies:

   ``catkin test --no-deps <package>`` will only run tests for the provided package.

:Forcing Test Discovery:

   ``catkin test --force-cmake <package>`` will force catkin to re-run cmake which will rerun the
   ``enable_bitbots_docs()`` cmake function which in turn will rerun test discovery.


Execute rostests manually
-------------------------

The *rostest* package provides a program with the name ``rostest`` which can be called manually to execute test
launch files.

It can be given a complete path like ``rostest ./test/rostests/test_something.launch`` or be called like *roslaunch*
``rostest my_package test_something.launch``.


Execute Python Tests via an external runner
-------------------------------------------

Python Unit Tests as well as rostests (the python file, not the launch file) can be run via any Python test runner
which supports the unittest framework (which should be all of them).

.. note:: When executing rostests without the *rostest* program, external dependencies which are defined in the
      tests launch file are not automatically started so you must do that manually.

Example runners and usages are:

:PyCharm:

   When opening a test files source code in PyCharm it will automatically recognize that this file contains tests
   and display green start buttons which can be pressed to add a run configuration that will execute this test.

   .. image:: ../_static/pycharm_test_file.png
       :alt: PyCharm editor window with the green buttons

:Using pytest:

   `pytest`_ is a test runner as well as extension to unittest. We are only interested in the test runner part though.

   Once installed it can be run like ``pytest test/unit_tests/test_something.py``.



Execute C++ tests manually
--------------------------

Executing C++ requires interaction with the build system since these tests need to be compiled and cannot simply
be executed like the ones in Python.
Still there are two ways to do it:

:Clion:

   Clion evaluates a packages ``CMakeLists.txt`` and knows about all targets defined in it and because tests are
   basically targets too, it knows about those.

   If you look at the available run configurations in Clion, you can see that some of them have a GoogleTest symbol.
   These are the targets for executing C++ Unit Tests and you can see an example in the image below.
   There will also be a run configuration called *All CTest* which will run **all** of the defined tests and not just
   C++ Unit Tests so it is probably not of much use. If you want to run all C++ Unit Tests there is a run configuration
   named like *run_tests_<package>_gtest* which does exactly that.

   .. image:: ../_static/clion_test_target.png
       :alt: Clion run configuration of a test target

:Catkin:

   Since running these tests requires interaction with the build system and catkin pretty much is the build system,
   C++ Unit Tests can of course also be executed using it.

   .. code-block:: bash

      catkin build -v --no-deps --force-cmake <package> --make-args run_tests_<package>_gtest


   :``-v``:

      is required to see the test output.

   :``--no-deps``:

      makes catkin only run for the given package.
   :``--force-cmake``:

      forces test discovery.

   :``--make-args``:

      overwrites the make target to the one which runs all C++ tests.


.. _pytest: https://pytest.org
