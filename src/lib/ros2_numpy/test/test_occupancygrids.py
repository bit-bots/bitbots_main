import unittest
import numpy as np
import ros2_numpy as rnp
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.serialization import serialize_message

class TestOccupancyGrids(unittest.TestCase):
    def test_masking(self):
        data = -np.ones((30, 30), np.int8)
        data[10:20, 10:20] = 100

        msg = rnp.msgify(OccupancyGrid, data)

        data_out = rnp.numpify(msg)

        self.assertIs(data_out[5, 5], np.ma.masked)
        np.testing.assert_equal(data_out[10:20, 10:20], 100)

    def test_serialization(self):
        msg = OccupancyGrid(
            info=MapMetaData(
                width=3,
                height=3
            ),
            data = [0, 0, 0, 0, -1, 0, 0, 0, 0]
        )

        data = rnp.numpify(msg)
        self.assertIs(data[1,1], np.ma.masked)
        msg2 = rnp.msgify(OccupancyGrid, data)

        self.assertEqual(msg.info, msg2.info)

        msg_ser = serialize_message(msg)
        msg2_ser = serialize_message(msg2)

        self.assertEqual(
            msg_ser,
            msg2_ser,
            "Message serialization survives round-trip")

if __name__ == '__main__':
    unittest.main()
