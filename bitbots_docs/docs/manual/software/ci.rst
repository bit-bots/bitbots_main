======================
Continuous Integration
======================
We have a Jenkins instance running at `ci.bit-bots.de <http://ci.bit-bots.de>`_ which automatically builds the current
documentation located at `doku.bit-bots.de <http://doku.bit-bots.de>`_.


Jenkins System Configuration
============================
Jenkins is configured in two levels.
One is the System-Configuration which is done via the Web-UI but contains only the bare minimum.
In short it contains:

:System Configuration: Only contains Security-Settings, which Github server to use, Jenkins base-url and
    user authentication setup.
:Credentials: Passwords, SSH-Keys and Api-Tokens to access source control
:bitbots_jenkins_library: This is a Jenkins Library (a collection of Groovy files) which can be loaded into
    a pipeline definition to provide common tasks.
    Its code is contained in `bit-bots/bitbots_jenkins_library <https://github.com/bit-bots/bitbots_jenkins_library>`_.
:Organisation Pipeline: This is the main Pipeline which builds most of our packages by indexing the Github Organisation.
    The pipeline type is actually GitHub Organisation as well.
    It is configured to search for repositories which contain a `Jenkinsfile` in its root folder and execute that
    Jenkinsfile.
:Non-Org Repo: Repositories which are not part of our Github Organisation and which are therefore not included in the
    above pipeline are configured separately. They are of the pipeline type `Multibranch Pipeline` because we
    might want to verify things before merging them into the master branch.

Jenkins is composed of a very small core and a metric ass-ton of plugins.
One of these plugins is the Github Plugin which lets Jenkins scan our `Organisation <https://github.com/bit-bots/>`_
for repositories with a valid configuration.


Repository configuration
========================
Each repository which uses CI contains a configuration-as-code file called ``Jenkinsfile``.
This file is located at the repository root and is written in Jenkins-DSL (domain specific language) a.k.a. Groovy.
All our `Jenkinsfiles` mostly look the same:

.. code-block:: groovy
    :linenos:

    @Library('bitbots_jenkins_library') import de.bitbots.jenkins.PackageDefinition

    bitbotsPipeline([
        new PackageDefinition("<package-name>", true),
        new PackageDefinition("<package-name2>", true)
    ] as PackageDefinition[])

Explained line by line:

1. First we use the Jenkins Library `bitbots_jenkins_library` and import the class `PackageDefinition` from it.
We will need both later.

3. We call `bitbotsPipeline` which is a function defined by the previously included Library.
It is the main Entrypoint of each of our CI Pipelines.
The function expects an array of `PackageDefinitions` for each of which it will execute a series of steps in
parallel.

4. Construction of a new PackageDefinition. This is a fairly staright-forward constructor call.


Build triggers
==============

Some builds are run periodically while most GitHub builds are triggered via Webhooks as well.
