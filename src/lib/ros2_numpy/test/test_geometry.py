import unittest
import numpy as np
import ros2_numpy as rnp
import tf_transformations as transformations

from geometry_msgs.msg import Vector3, Quaternion, Transform, Point, Pose

class TestGeometry(unittest.TestCase):
    def test_point(self):
        p = Point(x=1., y=2., z=3.)

        p_arr = rnp.numpify(p)
        np.testing.assert_array_equal(p_arr, [1, 2, 3])

        p_arrh = rnp.numpify(p, hom=True)
        np.testing.assert_array_equal(p_arrh, [1, 2, 3, 1])

        self.assertEqual(p, rnp.msgify(Point, p_arr))
        self.assertEqual(p, rnp.msgify(Point, p_arrh))
        self.assertEqual(p, rnp.msgify(Point, p_arrh * 2))

    def test_vector3(self):
        v = Vector3(x=1., y=2., z=3.)

        v_arr = rnp.numpify(v)
        np.testing.assert_array_equal(v_arr, [1, 2, 3])

        v_arrh = rnp.numpify(v, hom=True)
        np.testing.assert_array_equal(v_arrh, [1, 2, 3, 0])

        self.assertEqual(v, rnp.msgify(Vector3, v_arr))
        self.assertEqual(v, rnp.msgify(Vector3, v_arrh))

        with self.assertRaises(AssertionError):
            rnp.msgify(Vector3, np.array([0, 0, 0, 1]))

    def test_transform(self):
        t = Transform(
            translation=Vector3(x=1., y=2., z=3.),
            rotation=Quaternion(
                **dict(
                    zip(['x', 'y', 'z', 'w'],
                        transformations.quaternion_from_euler(np.pi, 0, 0))))
        )

        t_mat = rnp.numpify(t)

        np.testing.assert_allclose(
            t_mat.dot([0, 0, 1, 1]), [1.0, 2.0, 2.0, 1.0])

        msg = rnp.msgify(Transform, t_mat)

        np.testing.assert_allclose(msg.translation.x, t.translation.x)
        np.testing.assert_allclose(msg.translation.y, t.translation.y)
        np.testing.assert_allclose(msg.translation.z, t.translation.z)
        np.testing.assert_allclose(msg.rotation.x, t.rotation.x)
        np.testing.assert_allclose(msg.rotation.y, t.rotation.y)
        np.testing.assert_allclose(msg.rotation.z, t.rotation.z)
        np.testing.assert_allclose(msg.rotation.w, t.rotation.w)

    def test_pose(self):
        t = Pose(
            position=Point(x=1.0, y=2.0, z=3.0),
            orientation=Quaternion(
                **dict(
                    zip(['x', 'y', 'z', 'w'],
                    transformations.quaternion_from_euler(np.pi, 0, 0))))
        )

        t_mat = rnp.numpify(t)

        np.testing.assert_allclose(
            t_mat.dot([0, 0, 1, 1]), [1.0, 2.0, 2.0, 1.0])

        msg = rnp.msgify(Pose, t_mat)

        np.testing.assert_allclose(msg.position.x, t.position.x)
        np.testing.assert_allclose(msg.position.y, t.position.y)
        np.testing.assert_allclose(msg.position.z, t.position.z)
        np.testing.assert_allclose(msg.orientation.x, t.orientation.x)
        np.testing.assert_allclose(msg.orientation.y, t.orientation.y)
        np.testing.assert_allclose(msg.orientation.z, t.orientation.z)
        np.testing.assert_allclose(msg.orientation.w, t.orientation.w)

if __name__ == '__main__':
    unittest.main()
