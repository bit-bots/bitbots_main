# soccer_vision_attribute_msgs

This package provides messages for attributes for objects on a soccer field, such as balls, field markings and robots, that could be visually detected.

Such attributes can be defined separatly in ``soccer_vision_2d_msgs`` and ``soccer_vision_3d_msgs``, but due to large overlap in information, this package aims to abstract out the attributes that aren't specific to 2d/3d vision.

``soccer_vision_2d_msgs`` and ``soccer_vision_3d_msgs`` should depend on this package.

## Documentation

For documentation, see [Soccer Vision Attribute Msgs](https://soccer-interfaces.readthedocs.io/en/latest/soccer_vision_attribute_msgs.html).
