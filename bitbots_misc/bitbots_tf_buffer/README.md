# bitbots_tf_buffer

This is a drop-in replacement for `tf2_ros.TransformListener` in Python. It implements the callback for the
`/tf` and `/tf_static` topics in C++ to improve performance.

## Usage

Replace `from tf2_ros import TransformListener` with `from bitbots_tf_buffer import
TransformListener`.
