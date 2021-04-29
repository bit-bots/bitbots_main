======================
Continuous Integration
======================
We have a Jenkins instance running at `ci.bit-bots.de <http://ci.bit-bots.de>`_ which automatically builds
the current documentation located at `doku.bit-bots.de <http://doku.bit-bots.de>`_.

.. note:: If you only want to know what to do with your package and don't care about Jenkins internals you
    only need :ref:`enable_ci_for_repo` and :ref:`make_package_resolvable_in_ci`.


.. _enable_ci_for_repo:

Enable CI for a Repository
==========================
CI is enabled on a per-repository basis and not per-package.
Each repository which uses CI contains a configuration-as-code file called ``Jenkinsfile``.
This file is located at the repository root and is written in Jenkins-DSL (domain specific language) which
is based on Groovy which is based on Java.
All our `Jenkinsfiles` mostly look the same and utilizes common logic defined in our
`Pipeline Library <https://github.com/bit-bots/bitbots_jenkins_library>`_:

.. code-block:: groovy
    :linenos:

    @Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

    defineProperties()

    def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
    pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_docs")))
    pipeline.execute()


Explained line by line:

1. First we use the Jenkins Library `bitbots_jenkins_library` and import everything from it.

3. We call `defineProperties() <https://github.com/bit-bots/bitbots_jenkins_library/blob/master/vars/defineProperties.groovy>`_
   to set properties of this pipeline like when it is triggered and whether it may run concurrently.

5. We create a `pipeline` variable of type `BitbotsPipeline <https://github.com/bit-bots/bitbots_jenkins_library/blob/master/src/de/bitbots/jenkins/BitbotsPipeline.groovy>`_.
   This class contains most of the pipeline logic and is the main content of this library.

   By default, the pipeline restricts documentation publishing to only happen on the primary branch of a
   repository. Since we don't want to change the default behavior, we don't explicitly define the last
   argument.

6. We configure the pipeline to run for the package named *bitbots_docs*.
   See `PackagePipelineSettings <https://github.com/bit-bots/bitbots_jenkins_library/blob/master/src/de/bitbots/jenkins/PackagePipelineSettings.groovy>`_
   which configures what the pipeline should do and `PackageDefinition <https://github.com/bit-bots/bitbots_jenkins_library/blob/master/src/de/bitbots/jenkins/PackageDefinition.groovy>`_
   which defines a packages name and where it is defined in the repo.

   In this case, the pipeline should run for the package *bitbots_docs* which is located in a folder with the same name.
   The pipeline should *build the package*, *document the package* and *publish the package* which are the
   default settings so they are not explicitly given.


.. _make_package_resolvable_in_ci:

Make Package resolvable in CI
=============================
When package a is used as a dependency by package b and package b is supposed to have CI enabled, a needs
to be resolvable via rosdep.
This section assumes you are the maintainer of package a and want to make your package resolvable.

1. Create ``.rdmanifest`` file in your package (not repository) root by calling the script
   ``gen_rdmanifest.py`` in *bitbots_meta*. It's options should be self-explanatory when calling with
   `--help`.

   `.rdmanifest syntax reference <https://ros.org/reps/rep-0112.html#rdmanifest-syntax>`_

2. Define the package in our own `rosdep definition`_ by creating an entry like the following:

   .. code-block:: yaml

        bitbots_docs:
          ubuntu: &bitbots_docs
            source:
              uri: 'https://raw.githubusercontent.com/bit-bots/bitbots_tools/master/bitbots_docs/.rdmanifest'
          debian: *bitbots_docs
          fedora: *bitbots_docs




Jenkins System Configuration
============================
The System-Configuration is done via the Web-UI or via configuration-as-code from
`ansible <https://git.mafiasi.de/Bit-Bots/ansible/src/branch/master/host_vars/server/jenkins.yml>`_.
In short it contains:

:System Configuration: Only contains Security-Settings, which Github server to use, Jenkins base-url and
    user authentication setup.
:Credentials: Passwords, SSH-Keys and Api-Tokens to access source control
:bitbots_jenkins_library: This is a Jenkins Library (a collection of Groovy files) which can be loaded into
    a pipeline definition to provide common tasks.
    Its code is contained in `bit-bots/bitbots_jenkins_library <https://github.com/bit-bots/bitbots_jenkins_library>`_
    and it must be registered in the Jenkins configuration so that it is loadable in pipelines.

Jenkins is composed of a very small core and a ton of plugins.
One of these plugins is the Github Plugin which lets Jenkins scan our `Organisation <https://github.com/bit-bots/>`_
for repositories with a valid configuration and automatically registers it in Jenkins.


Build triggers
==============

Some builds are run periodically while most GitHub builds are triggered via webhooks as well.

These webhooks should be managed by the Github Plugin but in case that fails, Github must be configured
as follows:

:Payload URL: ``http://ci.bit-bots.de/github-webhook/``
:Content Type: ``application/json``
:Secret: Ask Finn or someone else who can see the current settings in Github.

    If that is not possible update the credential ``github-webhook-secret`` in Jenkins to a new value and set
    that in the webhook as well.
:Which events?: Send everything


Dependency Resolution Explained
===============================

Most of our packages have dependencies which are not normally resolvable via rosdep because they are our own
packages. For example, `bitbots_msgs` depends on `bitbots_docs` in order to build documentation.
To be able to resolve these dependencies while staying recent in their versions, a
`rosdep definition`_ has been
created which defines our packages as being installable via the `source` package manager.
Each package also has a ``.rdmanifest`` file in its package directory which then teaches rosdep how exactly
that package can be installed.
The `bitbots_builder <https://github.com/bit-bots/containers/tree/main/bitbots_builder>`_ docker image
(used by our CI) has this rosdep definition configured and is thus able to resolve our packages as
dependencies.

See `REP 111 <https://ros.org/reps/rep-0111.html>`_ and `REP 112 <https://ros.org/reps/rep-0112.html>`_
for more documentation about rosdep.


.. _rosdep definition: https://github.com/bit-bots/bitbots_tools/blob/master/rosdep_source.yml
