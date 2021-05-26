Troubleshooting
===============
Common errors and their solutions / workaround can be found here.

I can't open graphical applications while the tests are running
---------------------------------------------------------------
If the running tests utilize the webots simulator and have it running, this can occur.
It is caused by webots not really supporting a headless mode so we have resorted to starting it with a virtual
framebuffer (`Xvfb`_). This virtual framebuffer is actually a valid X server and some graphical programs try to connect
to it which fails because they are not authorized.

Unfortunately this is unavoidable behavior at the moment.

A workaround is, to start graphical applications before starting tests so that they connect to the real X server.


C++ Tests are always failing and don't get executed
---------------------------------------------------
If your C++ tests depend on the packages C++ source, all source files need to be given to ``enable_bitbots_tests()``
similar to the following example.
Include directories need not be given because they are added automatically via `include_directories()`_.

.. code-block:: cmake

   # CMakeLists.txt
   set(SOURCES src/bitbots_test.cpp)

   if (CATKIN_ENABLE_TESTING)
       find_package(catkin REQUIRED COMPONENTS bitbots_test)
       enable_bitbots_tests(CXX_SOURCES "${SOURCES}")
   endif()


Newly added test files don't get run
------------------------------------
When you add a test file (either Unit Test or rostest) they might not immediately get picked up by the build system
and don't get run when :doc:`executing tests <./executing_tests>`.

This is because most test execution will not re-execute CMake but test discovery is implemented in *bitbots_test* in
CMake.

To fix this, run your ``catkin test <package>`` call with ``--force-cmake``.
It should also be possible to call ``catkin build --force-cmake <package>``.


.. _Xvfb: https://en.wikipedia.org/wiki/Xvfb
.. _include_directories(): https://cmake.org/cmake/help/latest/command/include_directories.html

