Setup testing for a package
===========================

This guide describes how the *bitbots_test* library can be used in any package to make testing it easier.

#) First, *bitbots_test* needs to be added as a test dependency to your package.
   To do this, edit the *package.xml* file:

   .. code-block:: xml

      <package>
          <test_depend>bitbots_test</test_depend>
      </package>

#) Afterwards ``enable_bitbots_test()`` needs to be called in the *CMakeLists.txt* file of the package.
   This results in *bitbots_test* automatically discovering all tests and executing them when ``catkin test <package>``
   is called.

   .. code-block:: cmake

      if (CATKIN_ENABLE_TESTING)
          find_package(catkin REQUIRED COMPONENTS bitbots_test)
          enable_bitbots_tests()
      endif()

   This function is defined in `enable_bitbots_test.cmake.in`_.

   .. todo:: Add exact documentation for the ``enable_bitbots_test`` function.


#) Call ``catkin build --make-args test -- --no-deps --force-cmake <package>`` to create the necessary directories
   in which tests are located.

   This can also be done manually.
   Simply create *test/rostests* and *test/unit_tests* directories.

   .. seealso:: :doc:`./writing_tests` on how to write tests once testing is enabled.


.. _enable_bitbots_test.cmake.in: https://github.com/bit-bots/bitbots_tools/blob/master/bitbots_test/cmake/enable_bitbots_tests.cmake.in