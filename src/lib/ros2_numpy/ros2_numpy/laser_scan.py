#!/usr/bin/env python3
"""
Methods to numpify LaserScan message.
"""

from .registry import converts_from_numpy, converts_to_numpy

import numpy as np
from sensor_msgs.msg import LaserScan


@converts_to_numpy(LaserScan)
def laserscan_to_array(
    scan, remove_invalid_ranges=False, include_ranges_and_intensities=False
):
    """
    Takes a sensor_msgs/msg/LaserScan msg and returns a structered array with
    fields x, y and z that correspond to cartesian position data. Optionally,
    ranges and intensities fields that correspond to the range and intensity
    of a point are also included if include_ranges_and_intensities is True.

    Parameters
    ----------
    scan : ROS2 LaserScan message
        Input laser scan message to get numpyed
    remove_invalid_ranges : bool, optional
        whether to remove invalid ranges from the input scan, by default False
    include_ranges_and_intensities : bool, optional
        whether to also return the ranges & intensities along with the cartesian
        position.

    Returns
    -------
    pts : numpy k-array where each element will be a structured record that
        contains either 3 or 5 fields. If include_ranges_and_intensities is
        False, each element of  output array is a structured record that has
        3 fields ['x', 'y', 'z'] of type float 32. Else, it has 5 fields ['x',
        'y', 'z', 'ranges', 'intensities']. Since output is a structured array,
        all the x-coordinates of the points can be accessed as out_array['x'].
        Similarly, the y and z coodinates can be accessed as out_array['y']
        and out_array['z'] respectively.
    """
    n_points = len(scan.ranges)
    angles = np.linspace(
        scan.angle_min,
        scan.angle_max,
        n_points,
    )
    ranges = np.array(scan.ranges, dtype="f4")
    intensities = np.array(scan.intensities, dtype="f4")
    if remove_invalid_ranges:
        indices_invalid_range = (
            np.isinf(ranges)
            | np.isnan(ranges)
            | (ranges < scan.range_min)
            | (ranges > scan.range_max)
        )
        ranges = ranges[~indices_invalid_range]
        angles = angles[~indices_invalid_range]
        intensities = intensities[~indices_invalid_range]

    x = np.array(ranges * np.cos(angles), dtype="f4")
    y = np.array(ranges * np.sin(angles), dtype="f4")
    z = np.zeros(ranges.shape[0], dtype="f4")
    if include_ranges_and_intensities:
        dtype = np.dtype(
            [
                ("x", "f4"),
                ("y", "f4"),
                ("z", "f4"),
                ("ranges", "f4"),
                ("intensities", "f4"),
            ]
        )
        out_array = np.empty(len(x), dtype=dtype)
        out_array["x"] = x
        out_array["y"] = y
        out_array["z"] = z
        out_array["ranges"] = ranges
        out_array["intensities"] = intensities
        return out_array
    else:
        dtype = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4")])
        out_array = np.empty(len(x), dtype=dtype)
        out_array["x"] = x
        out_array["y"] = y
        out_array["z"] = z
        return out_array


@converts_from_numpy(LaserScan)
def array_to_laserscan(arr, header, scan_time=0.0, time_increment=0.0):
    """
    Takes a structured array(created from LaserScan msg) and returns a
    LaserScan message. Fields that cannot be determined from the numpy
    array are provided as inputs. Since the LaserScan message relies
    on a consistent angular increment, structured array in which the
    points have been omitted will result in a LaserScan message that is
    not correct.

    Parameters
    ----------
    arr : Structured numpy array with fields x, y and z.
        Input numpy array that was created from LaserScan message.
    header : std_msgs::msg::Header,
        The header to be written to the output LaserScan message.
    scan_time : float, optional
        time between scans [seconds] of LaserScan
    time_increment : float, optional
        time between measurements [seconds] - if lidar is moving,
        this will be used in interpolating position

    Returns
    -------
    scan_msg : sensor_msgs::msg::LaserScan message
        If the input array does not contain an `intensities` field,
        this message has the intensities list filled to zeros.

    """
    n_points = arr.shape[0]

    if "intensities" in arr.dtype.names:
        intensities = arr["intensities"]
    else:
        intensities = np.zeros(n_points).astype(float)

    if "ranges" in arr.dtype.names:
        ranges = arr["ranges"]
    else:
        ranges = np.sqrt(arr["x"] ** 2 + arr["y"] ** 2).astype(float)

    angles = np.arctan2(arr["y"], arr["x"]).astype(float)

    # Create a LaserScan message
    scan_msg = LaserScan()

    scan_msg.header = header

    scan_msg.intensities = intensities.tolist()
    scan_msg.ranges = ranges.tolist()

    # Compute min and max of the ranges
    scan_msg.range_min = np.min(ranges)
    scan_msg.range_max = np.max(ranges)

    # Compute min and max of the angles
    scan_msg.angle_min = np.min(angles)
    scan_msg.angle_max = np.max(angles)

    # Use the time_increment and scan_time from input arguments
    scan_msg.time_increment = time_increment
    scan_msg.scan_time = scan_time

    # Compute angle increment. Since the angle_max is not exclusive, omit
    # the last point.
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (
        n_points - 1
    )

    return scan_msg
