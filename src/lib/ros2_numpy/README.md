# ros2_numpy
This project is a fork of [ros_numpy](https://github.com/eric-wieser/ros_numpy)
to work with ROS 2. It provides tools for converting ROS messages to and from
numpy arrays. In the ROS 2 port, the module has been renamed to
`ros2_numpy`. Users are encouraged to update their application code to import
the module as shown below.

ROS 2:

```
import ros2_numpy as rnp
```

ROS 1:

```
import ros_numpy as rnp
```

Prefacing your calls like `rnp.numpify(...)` or `rnp.msgify(...)` should help
future proof your codebase while the ROS 1 and ROS 2 ports are API compatible.

The ROS 2 port has been bootstrapped as version `2.0.3`. The `MAJOR`
version has been set to `2` to indicate ROS 2 and the `MINOR` and `PATCH`
versions match the ROS 1 version from which the ROS 2 port was
bootstrapped. The reasoning behind this is to allow for creating tags in this
fork that can be released into the ROS 2 distribution while not conflicting
with existing tags on the upstream repository. A release into Foxy is still
pending.

This module contains two core functions:

* `arr = numpify(msg, ...)` - try to get a numpy object from a message
* `msg = msgify(MessageType, arr, ...)` - try and convert a numpy object to a message

Currently supports:

* `sensor_msgs.msg.PointCloud2` &harr; structured `np.array`:

   ```python
   data = np.zeros(100, dtype=[
     ('x', np.float32),
     ('y', np.float32),
     ('vectors', np.float32, (3,))
   ])
   data['x'] = np.arange(100)
   data['y'] = data['x']*2
   data['vectors'] = np.arange(100)[:,np.newaxis]

   msg = ros2_numpy.msgify(PointCloud2, data)
   ```

   ```
   data = ros2_numpy.numpify(msg)
   ```

* `sensor_msgs.msg.LaserScan` &harr; structured `np.array`
* `sensor_msgs.msg.Image` &harr; 2/3-D `np.array`, similar to the function of `cv_bridge`, but without the dependency on `cv2`
* `nav_msgs.msg.OccupancyGrid` &harr; `np.ma.array`
* `geometry.msg.Vector3` &harr; 1-D `np.array`. `hom=True` gives `[x, y, z, 0]`
* `geometry.msg.Point` &harr; 1-D `np.array`. `hom=True` gives `[x, y, z, 1]`
* `geometry.msg.Quaternion` &harr; 1-D `np.array`, `[x, y, z, w]`
* `geometry.msg.Transform` &harr; 4&times;4 `np.array`, the homogeneous transformation matrix
* `geometry.msg.Pose` &harr; 4&times;4 `np.array`, the homogeneous transformation matrix from the origin

Support for more types can be added with:

```python
@ros2_numpy.converts_to_numpy(SomeMessageClass)
def convert(my_msg):
    return np.array(...)

@ros2_numpy.converts_from_numpy(SomeMessageClass)
def convert(my_array):
    return SomeMessageClass(...)
```

Any extra args or kwargs to `numpify` or `msgify` will be forwarded to your conversion function


## Future work

* Add simple conversions for:

  * `geometry_msgs.msg.Inertia`
