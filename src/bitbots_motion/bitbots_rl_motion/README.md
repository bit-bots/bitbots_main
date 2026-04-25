## General

The package contains a framework which is a capsule for the application of policies on a robot.

## Framework structure

The code is divided in five sections: Configs, Handlers, Nodes, Launch and rest.

The Nodes-folder contains all relevant ROS-Nodes regarding policy models. These nodes are responsible for starting the policies correctly, feeding them with correct data, defining in which robot states they are allowed to publish and publishing their outputs correctly.
The name of the node describes for which kind of policy it is suitable. 
The RL Node is a special case. All other nodes are kids of the RL Node. It centralizes the execution loop and minimizes boiler plate code. 

The Handlers-folder contains all handlers. A handler is a specific type of object which is responsible for processing (subscribed) external data such that they are comprehensible for the policy models. All handlers are kids of the Handler class.

The Configs-folder contains all robot/policy specific configurations. Files in the Configs-folder should be in .yaml-format. They also contain the paths to the onnx-policy models.

The Launch-folder contains a launch file which starts all relevant policy nodes.

phase.py and previous_action.py are two files, which do not fall in any of the aforementioned categories. 
phase.py defines a PhaseObject, which is responsible for the phase management. previous_action.py defindes a PreviousAction object, which is responsible for saving and provide the previous action.
Both files are located in the bitbots_rl_motion folder. 

## Execution

For proper starting you need a policy model and a config file. The config file should have the same structure as the wolfgang_dribbling_model_config.yaml file.
Furthermore, you have to create or adjust a node file to your needs. walk_node.py can be used for orientation. If chages are conducted on the RL_Node class, it should be announced.
Finally, you define which nodes and policies you wanna use in the launch file.

## Testing 

Tests are designed to check basic functionality of the existing models and import correctness of the config files. 
MyPi test checks whether type errors exist.
