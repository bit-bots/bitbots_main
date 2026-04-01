## General

The package contains a framework which is a capsule for the application of policies on a robot.

## Framework structure

The code is divided in five sections: Configs, Handlers, Models, Nodes and rest.

The Models-folder contains all policy models. Node, the framework expects that policies are in .onnx-format. 

The Nodes-folder contains all relevant ROS-Nodes regarding policy models. These nodes are responsible for starting the policies correctly, feeding them with correct data and publishing their outputs correctly.
The name of the node describes for which kind of policy it is suitable. 
The RL Node is a special case. All other nodes are kids of the RL Node. It centralizes the execution loop and minimizes boiler plate code. 

The Handlers-folder contains all handlers. A handler is a specific type of object which is responsible for processing external data such that they are comprehensible for the policy models. All handlers are kids of the Handler class.

The Configs-folder contains all robot/policy specific configurations. Files in the Configs-folder should be in .yaml-format.

phase.py, policy_nodes.py and previous_action.py are two files, which do not fall in any of the aforementioned categories. 
phase.py defines a PhaseObject, which is responsible for the phase management. previous_action.py defindes a PreviousAction object, which is responsible for saving and provide the previous action. policy_nodes.py can be looked as the launch file. It defines, which nodes and policies will be used. 
Both files are located in the bitbots_rl_motion folder. 

## Execution

For proper starting you need a policy model and a config file. The config file should have the same structure as the wolfgang_config.yaml file.
Furthermore, you have to create or adjust a node file to your needs. walk_node.py can be used for orientation. If chages are conducted on the RL_Node class, it should be announced.
Finally, you define which nodes and policies you wanna use in the policy_nodes.py file.  




