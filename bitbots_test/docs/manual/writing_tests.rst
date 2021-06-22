Writing Tests
=============
This guide describes how new test cases can be created as well as some best practices to follow.

Most packages which use *bitbots_test* will have a :file:`test/` folder in which all test code resides.
This folder is further divided between the different kinds of tests that *bitbots_test* supports.
Currently there are two kinds of tests:

:Unit Tests:

   Unit tests are designed to extensively test small components or units of a larger system.
   *bitbots_test* supports unit tests written in :ref:`Python <writing_unit_tests_in_python>` as well as ones written in
   :ref:`C++ <writing_unit_tests_in_cxx>`.
   They are located (by default) in :file:`test/unit_tests`.

:rostests:

   rostests are basically launch files which contain one or more ``<test>`` tags.
   They can be used for package tests that require interacting with ROS as well as integration or system tests.
   *bitbots_test* only supports :ref:`writing rostests <writing_rostests>` in Python.
   They are located (by default) in :file:`test/rostests`.


.. _writing_unit_tests_in_python:

Writing Unit-Tests in Python
----------------------------

#) Add a file like :file:`test/unit_tests/test_{something}.py`.
   These test files should not be executable.

#) Create one or more classes derived from :class:`bitbots_test.test_case.TestCase` in that file.

#) Create one ore more methods in your class prefixed with ``test_``.
   When executing tests, each of these methods will be called as a separate test.

   .. seealso:: :doc:`./executing_tests` on how to do the executing.

#) Call one of the provided ``assert`` methods defined on your derived class.

   The easiest way to see which assertions are available is through IDE autocomplete.
   Otherwise some documentation is available through the :class:`unittest.TestCase` documentation and the
   various assertion mixins in :mod:`bitbots_test.test_case`.
   Which assertion exactly are available depends on the *TestCase* type you are subclassing though (i.e. simulator
   assertions are only available when subclassing :class:`bitbots_test.test_case.WebotsTestCase`).

Here is an example:

.. code-block:: python3

   from bitbots_test.test_case import TestCase

   class ExampleTestCase(TestCase):
       def test_equals(self):
           self.assertEqual(1, 1)

       def test_list_contains(self):
           my_list = [i for i in range(0, 10) if i % 2 == 0]
           self.assertIn(4, my_list)


.. _writing_unit_tests_in_cxx:

Writing Unit-Tests in C++
-------------------------

#) Add a file like :file:`test/unit_tests/test_{something}.cpp`.

#) Include ``<bitbots_test.h>`` as well as ``<gtest/gtest.h>``

#) Add tests using one of the ``TEST`` macros and assert behavior using one of the ``EXPECT`` or ``ASSERT`` macros.

   The difference between the two verification macros is that *assertions* exit the current test function and thus
   skip potential cleanup code which may create memory leaks. Because of that *expect* based verifications are
   preferred.

#) Define a main function which initializes and runs all tests.

.. seealso:: Since most functionality is fairly standard GoogleTest behavior see the `GoogleTest User Guide`_ for
      more detailed information

Here is an example:

.. code-block:: C++

   #include "bitbots_test.h"
   #include "gtest/gtest.h"

   TEST(ExampleTestCase, TestEquals) {
       EXPECT_EQ(1, 1)
   }

   TEST(ExampleTestCase, TestStringUnEquality) {
       EXPECT_STRNE("hello", "world")
   }

   int main(int argc, char **argv) {
       testing::InitGoogleTest(&argc, argv);
       return RUN_ALL_TESTS();
   }


.. _writing_rostests:

Writing rostests
----------------

#) Create a launch file like :file:`test/rostests/test_{something}.launch` and a corresponding python file beside
   it like :file:`test/rostests/test_{something}.py`.
   The Python file should be executable because it will later be started by rostest as a node.

#) The launch file uses normal `Launch File Syntax`_ but the relevant parts are that *rostest* will start
   nodes defined using the `Test Tag`_ instead of ignoring it like *roslaunch*.

   This means that you should define at least one ``<test>`` node inside it but you can also bring up all dependencies
   necessary for your tests.

   Here is an example:

   .. code-block:: xml

      <launch>
          <test pkg="your_package" type="test_something.py" test-name="test_something"/>
      </launch>

#) The python file which we created earlier and which is launched as a ``test`` is similar in structure to Unit Tests
   written in Python.

   In essence, they contain:

   #) The Shebang ``#!/usr/bin/env python3`` at the top of the file

   #) One ore more classes which inherit from :class:`bitbots_test.test_case.RosNodeTestCase`.
      These classes should contain your tests as methods prefixed with ``test_``.

      .. note:: Python Unit Tests inherit from :class:`bitbots_test.test_case.TestCase` while rostests inherit from
         from :class:`bitbots_test.test_case.RosNodeTestCase`. The difference between the two is that
         *RosNodeTestCase* implements some additional assertions and manages the tests rosnode lifecycle
         automatically.

   #) A call to :func:`bitbots_test.run_rostests` to which all of your test case classes are passed.

   Here is an example:

   .. code-block:: python3

      #!/usr/bin/env python3
      from bitbots_test.test_case import RosNodeTestCase

      class ExampleTestCase(RosNodeTestCase):
          def test_roslog(self):
              # assert that nothing at all is logged
              self.assertNotRosLogs()

      if __name__ == "__main__":
          from bitbots_test import run_rostests
          run_rostests(ExampleTestCase)


.. _general_test_structure:

General Test structure
----------------------

Most tests can be split into the three steps *setup*, *execution* and *verification* although not all of these steps
are always necessary. If your tests are in any way non-trivial they should reflect this structure because that makes
it easier to understand the test code later.

Here is an example:

.. code-block:: python3

   def test_some_publisher(self):
       # setup
       pub = rospy.Publisher("/test", std_msgs.msg.Empty, queue_size=10)
       sub = MockSubscriber("/test", std_msgs.msg.Empty, queue_size=10)
       sub.wait_until_connected()

       # execution
       pub.publish(std_msgs.msg.Empty())

       # verification
       sub.assertMessageReceived()


.. _`GoogleTest User Guide`: https://google.github.io/googletest/
.. _Launch File Syntax: https://wiki.ros.org/roslaunch/XML
.. _Test Tag: https://wiki.ros.org/roslaunch/XML/test
