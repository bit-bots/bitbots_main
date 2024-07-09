# bitbots_tf_buffer

This is a nearly drop-in replacement for `tf2_ros.Buffer` in Python. It wraps a C++ node that holds the tf buffer and listener. The interface should be the same as the original `tf2_ros.Buffer`, except that we need to pass a reference to the node to the constructor.

## Usage

- Replace `from tf2_ros import Buffer, TransformListener` with `from bitbots_tf_buffer import
Buffer`.
- Remove the `TransformListener` from the code
- Pass a reference to the node to the constructor of `Buffer`:

```python
from bitbots_tf_buffer import Buffer

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.tf_buffer = Buffer(self)
```
