Optimization with Optuna
========================

We use Optuna_ to automatically optimize many parameters, e.g. for walking.
You can find detailed information on how this works in their documentation.
We implemented the package parallel_parameter_search_ that provides some basic classes and examples on how to use it with our software stack.
The general concept is, that a set of parameters is suggested by optuna.
This is then evaluated in simulation, e.g. by letting the walk forward.
Afterwards, the performance of this parameter set is evaluated, e.g. by seeing how stable the robot walked.
This is then repeated for a given number of times.
During this, Optuna uses the ratings of already tried out parameters to suggest better parameters.

.. _Optuna: https://optuna.readthedocs.io/en/stable/
.. _parallel_parameter_search: https://github.com/bit-bots/parallel_parameter_search