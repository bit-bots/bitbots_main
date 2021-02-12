# Bit-Bots containers
A collection of docker containers used in various capacities.

- [jenkins](./jenkins) A periodically rebuilt container that includes all plugins needed by our Jenkins instance 
    ([ci.bit-bots.de](http://ci.bit-bots.de)).
- [bitbots_builder](./bitbots_builder) A container image which is able to compile and document our codebase.
    It exists so that we don't need to reinstall all dependencies whenever a build is triggered.
