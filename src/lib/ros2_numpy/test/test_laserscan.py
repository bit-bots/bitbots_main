import unittest
import numpy as np
import ros2_numpy as rnp

from sensor_msgs.msg import LaserScan
from rclpy.clock import Clock


class TestLaserScan(unittest.TestCase):
    def test_to_and_from_laserscan(self):
        # Create a dummy LaserScan message
        scan = LaserScan()
        scan.header.stamp = Clock().now().to_msg()
        scan.header.frame_id = "lidar_2d"
        scan.range_min = 0.01
        scan.range_max = 200.0
        scan.angle_increment = np.radians(0.1)
        scan.angle_min = -np.pi
        scan.angle_max = np.pi - scan.angle_increment
        scan.scan_time = 0.0
        scan.time_increment = 0.0
        scan.ranges = np.full(3600, 10.0, dtype="f4").tolist()
        scan.intensities = np.full(3600, 5, dtype="f4").tolist()
        laserscan_array = rnp.numpify(
            scan,
            remove_invalid_ranges=False,
            include_ranges_and_intensities=True,
        )

        self.assertEqual(laserscan_array.shape[0], 3600)
        np.testing.assert_array_equal(
            laserscan_array["ranges"], np.array(scan.ranges)
        )
        np.testing.assert_array_equal(
            laserscan_array["intensities"], np.array(scan.intensities)
        )

        laserscan_array_without_ranges_and_intensities = rnp.numpify(scan)

        laserscan_msg = rnp.msgify(
            LaserScan,
            laserscan_array_without_ranges_and_intensities,
            scan.header,
            scan.scan_time,
            scan.time_increment,
        )
        self.assertAlmostEqual(
            laserscan_msg.angle_min, scan.angle_min, places=5
        )
        self.assertAlmostEqual(
            laserscan_msg.angle_increment, scan.angle_increment
        )
        self.assertAlmostEqual(laserscan_msg.angle_max, scan.angle_max)
        self.assertAlmostEqual(laserscan_msg.scan_time, scan.scan_time)
        self.assertAlmostEqual(
            laserscan_msg.time_increment, scan.time_increment
        )
        self.assertAlmostEqual(laserscan_msg.header.stamp, scan.header.stamp)
        self.assertEqual(laserscan_msg.header.frame_id, scan.header.frame_id)
        self.assertEqual(len(laserscan_msg.ranges), 3600)
        self.assertEqual(len(laserscan_msg.intensities), 3600)
        np.testing.assert_array_equal(
            laserscan_array["ranges"], np.array(scan.ranges)
        )


if __name__ == "__main__":
    unittest.main()
