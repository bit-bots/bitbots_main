Coding Style
============

Because we value maintainability and readability of our codebase, we have chosen to program according to the following
style guides.

Python
------

Our Python guide is based on `Google's style <http://google.github.io/styleguide/pyguide.html>`_ with the following
adaptions:

* **3.2** - Instead of an 80 character maximum line length we do 120 characters.
* **3.8.2** - Since we have a _LICENSE_ file in every repository we refrain from including license boilerplate in every file.
* as a docstring format we use `reST <https://www.python.org/dev/peps/pep-0287/>`_.

PyCharm Integration
~~~~~~~~~~~~~~~~~~~

* Go to File > Settings > Editor > Code Style. Click the gear icon and select Import Scheme > IntelliJ Idea code style XML.
  Download :download:`our python style</_static/bitbots_python_style.xml>` and select it. Choose a name to store it.
* Make sure to select all python code inspections in Editor > Inspections > Python.

VSCode Integration
~~~~~~~~~~~~~~~~~~
Install the Python extension.
Add the following lines to your setting.json ($HOME/.config/Code/User/settings.json)

.. code-block:: json

   "python.formatting.provider": "yapf",
   "python.formatting.yapfArgs": [
       "--style={based_on_style: google, column_limit: 120}"
   ],
   "editor.formatOnType": true,
   "editor.formatOnPaste": true,



C++
---

Our C++ guide is based on the `ROS C++ Style Guide <http://wiki.ros.org/CppStyleGuide>`_ except that our ``{`` are not
on a new line.

CLion Integration
~~~~~~~~~~~~~~~~~~~

* Go to File > Settings > Editor > Code Style. Click the gear icon and select Import Scheme > IntelliJ Idea code style XML.
  Download :download:`our cpp style</_static/bitbots_cpp_style.xml>` and select it. Choose a name to store it.
* Make sure to select all python code inspections in Editor > Inspections > C/C++.

VSCdoe Integration
~~~~~~~~~~~~~~~~~~
Install the C/C++ extension.
Add the following lines to your setting.json ($HOME/.config/Code/User/settings.json)

.. code-block:: json

   "C_Cpp.clang_format_fallbackStyle": "{ BasedOnStyle: Google}",
   "editor.formatOnType": true,
   "editor.formatOnPaste": true,


Git
---

We also have some conventions about how we want to use git. They are as follows:

* Commits are written like ``when this commit is applied, it will <commit message>``.
* Commits consist of a heading, newline and optional description.
    * the heading is a short summary of the changes.
    * the heading should start with a prefix like ``ci:``, ``walking:``, etc to show which part of the repository is affected.
    * the description should not elaborate further on what was done but rather explain why something was done.
* Any change should be done in a branch and then PRed to master.
    * branch names should be prefixed with ``feature/``, ``fix/``, ``refactor/`` or none if not applicable.
    * merged branches can be deleted to keep the repo clean of any clutter.
