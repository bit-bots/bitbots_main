# dynamic_stack_decider

A lightweight framework for decision making and behavior defining for robots and other agents in changing environments.
It combines the advantages of behavior trees, decision trees and state machines.

## General Structure

In general, the dynamic stack decider consists of actions and decisions.
A decision is a python class extending `dynamic_stack_decider.AbstractDecisionElement`.
It should implement the `perform()` method where, depending on some information, a string is returned.

An action is a python class extending `dynamic_stack_decider.AbtractActionElement`.
It should also implement the `perform()` method where code can be executed that performs an action.
It should not return anything.

The DSD follows a tree-like structure that is specified in the DSD domain-specific language.
Let's look at an example:

```
-->
$Mode
    BALL --> $BallSeen
        YES --> @TrackBall
        NO --> @SearchBall
    PATTERN --> @LookAround
```

This example is a very simple head behavior for a robot.
The arrow (`-->`) at the beginning of the file indicates the start of the behavior.
Decisions are prefix with `$`, actions with `@`.
Their names correspond to the class names of the python classes implementing the respective action or decision.

The first decision is the `Mode` decision.
It decides about the mode of the head and returns the string `BALL` or `PATTERN`.
By convention, the returned strings are in capital letters.
The returned strings are indented and followed by an arrow; the element after the arrow is executed when the corresponding string is returned by the decision.

<p align="center">
<img src="dynamic_stack_decider/docs/_static/DSD_Stack_Example.png" alt="A visualization of the Stack" width="350px"/>
</p>

The underlying model for the DSD is a stack.
The root element (`Mode` in the example above) is always at the bottom of the stack.
When it makes a decision, the result is pushed on the stack and executed, for example the `BallSeen` decision.
Then, depending on the result, the next decision or action is pushed and executed, for example the `TrackBall` action.
This actions stays on top of the stack until one of the following conditions is met:
1. It calls `self.pop()` to pop itself from the stack.
   Then, the decision that is now on top of the stack is executed again and depending on the result, another action or decision is pushed.
2. One of the decisions on the stack is reevaluated (see #Reevaluation below).

The `__init__` method of an element is called when it is pushed on the stack.
It receives the following arguments:
* `blackboard`: an arbitrary object that is shared between all stack elements
* `dsd`: the instance of the dynamic stack decider
* `parameters`: an optional dictionary of parameters

The `perform` method is called when it is on top of the stack or getting reevaluated.
It receives the argument `reevaluate` which is set to `True` when the element is getting reevaluated and to `False` when it is executed normally.

## Starting the DSD

To start the DSD, a `dynamic_stack_decider.DSD` object has to be created.
Its arguments are a blackboard, which is passed to all elements, and an optional ROS topic that is used for debugging.
Then, `register_actions` and `register_decisions` have to be called with the name of a folder containing the actions and decisions, respectively.
After registering the elements, `load_behavior` should be used to load the file containing the behavior description in the DSD language.
Finally, to actually execute the stack decider, `update` should be called repeatedly.

On each update, all elements on the stack except the top elements are reevaluated from bottom to top, if requested.
Then, the top element is executed.
If it was a decision, the resulting action is pushed on the stack, to be executed in the next call to `update`.

## Reevaluation

Normally, only the topmost action or decision of the stack is executed and another decision can only be executed if the topmost element pops itself from the stack.
Sometimes, it is useful to also execute particular decisions when they are not on top of the stack.
For example in the example above, if the mode can be set externally, it would make sense to check on every update of the stack if the decision has changed.
To achieve that behavior, the `get_reevaluate` should be implemented, either to just return `True` or to implement more sophisticated reevaluation criteria.

When the element is reevaluated, `perform` is called on the element.
If the result of `perform` is the same as the last time it was called, nothing happens and the next decision is reevaluated.
If the result of `perform` is different from the last time, everything above the reevaluated decision is discarded and a new element, depending on the result of `perform`, is pushed on the stack and executed.

An action can call `self.do_not_reevaluate()` to avoid reevaluation of the stack on the next call to `update`. Alternatively the parameters (see #Parameters) `r` or `reevaluate` can be used in the dsd file to enable or disable the reevaluation during a specific action.

## Sequence elements

Sequence elements can be used to perform multiple actions instead of a single one.
To use a sequence element, simply separate the actions with commas, like that: `@FirstAction, @SecondAction, @ThirdAction`.
When the sequence element is pushed on the stack, the first action of the sequence remains on top of the stack until it pops itself from the stack.
Then, the next action in the sequence is pushed to the stack.

## Parameters

Sometimes it is helpful to pass parameters to actions or decisions.
The syntax is `@Action + parameter:value`, multiple parameters can be passed using further `+ parameter:value` pairs.
The parameters to an element are passed to the element's constructor as a dictionary.
The value of the parameter is passed through `yaml.safe_load`.
Therefore, you can use integers, floats, booleans or strings as values.
It is also possible to load parameters from the ROS parameter server using `parameter:%rosparam`, where `rosparam` is the name of the parameter on the parameter server.

Here is an example:
```
-->
$Mode
    BALL --> $BallSeen
        YES --> @TrackBall + time:10
        NO --> @SearchBall
    PATTERN --> @LookAround
```

## Subtrees

To avoid duplication and deep nested structures in the DSD file, subtrees can be used.
Here is an example that is equivalent to the example above:
```
#BallMode
$BallSeen
    YES --> @TrackBall
    NO --> @SearchBall

-->
$Role
    BALL --> #BallMode
    PATTERN --> @LookAround
```

Subtrees can also receive parameters.
Here is another example:
```

#BallMode
$BallSeen + tracktime
    YES --> @TrackBall + time:*tracktime
    NO --> @SearchBall

-->
$Role
    BALL --> #BallMode + tracktime:10
    PATTERN --> @LookAround
```

As you can see, the parameters are specified in the same way as for actions or decisions.
In the subtree, the parameter has to be given a name using the `+` syntax and can be referenced with the `*` operator.

## ELSE

`ELSE` is a catch-all decision result.
It can be used to map multiple results to a single action or decision.
For example, the `Role` decision could return the roles `BALL`, `GOAL`, `ROBOT`, and `PATTERN`, but the latter three should result in the same behavior:
```
-->
$Mode
    BALL --> $BallSeen
        YES --> @TrackBall
        NO --> @SearchBall
    ELSE --> @LookAround
```

## Interrupt

A stack element can call `self.interrupt()` to create an interrupt which removes all elements except for the start element from the stack (i.e., the DSD is restarted).

## Debugging

For debugging, the DSD visualization should be used.
It automatically discovers all running DSDs with a debug topic and visualizes them in RQT.
The visualization shows the DSD tree with the currently active actions and decisions as a tree on the left and the current stack on the right.
More information can be added to the right side by calling `self.publish_debug_data(label, data)` in any action or decision.

## DSD Development GUI
If you want to use a graphical user interface to define your DSD, we recommend [this repository](https://github.com/bit-bots/dsd-gui)

## VSCode Extension
If you use VSCode, you can use the [DSD extension](https://marketplace.visualstudio.com/items?itemName=Mastermori.dsd) which provides syntax highlighting.

## Examples

Here are a few projects that use the DSD and can be used for reference:
* The [Bit-Bots Behavior](https://github.com/bit-bots/bitbots_main/tree/main/bitbots_behavior)
* The [Bit-Bots Humanoid Control Module](https://github.com/bit-bots/bitbots_main/tree/main/bitbots_motion/bitbots_hcm/bitbots_hcm/hcm_dsd)
* The [Parser Test](https://github.com/bit-bots/dynamic_stack_decider/tree/master/dynamic_stack_decider/test) in this repository

## The Paper

The framework is also described in more depth in [DSD – Dynamic Stack Decider](https://link.springer.com/article/10.1007/s12369-021-00768-8)
