RL Motion
=========

This package provides ROS nodes that execute motions that were previously learned via reinforcement learning.
The node uses model from the 'models' folder based on the config in the 'config'.

Using a New Model
-----------------

The model can be trained using the code from the https://github.com/bit-bots/deep_quintic git.
The training script will generate a folder in the specified log location.
You need to choose which saved model (either the best model or one of the checkpoints) should be used.
Rename the corresponding file to 'model.zip'.
Copy it to a new folder in the 'models' folder together with the 'args.yml', 'config.yml' and vecnormalize.pkl'.
Change the config to point to your new model.
See the documentation in the git for more information on training models.