import unittest
from bitbots.modules.behaviour.body.actions.go_to_absolute_position import GoToAbsolutePosition


class TestGoToAbsolutePosition(unittest.TestCase):
    def test_determine_angle_to_turn(self):
        gtap = GoToAbsolutePosition([-4500, 0, 0])

        current_x = 0
        current_y = 0
        vector_to_target, radius, angle = gtap.determine_vector_and_angle_towards_target(current_x, current_y)
        self.assertEquals(180, angle)
        self.assertEquals(4500, radius)
        self.assertEquals([-4500, 0], vector_to_target)

        current_x = -3000
        current_y = 1500
        vector_to_target, radius, angle = gtap.determine_vector_and_angle_towards_target(current_x, current_y)
        self.assertEquals(225, angle)
        self.assertEquals((1500 ** 2 + 1500 ** 2) ** 0.5, radius)
        self.assertEquals([-1500, -1500], vector_to_target)

        gtap = GoToAbsolutePosition([0, 0, 0])
        current_x = -4500
        current_y = 0
        vector_to_target, radius, angle = gtap.determine_vector_and_angle_towards_target(current_x, current_y)
        self.assertEquals(0, angle)
        self.assertEquals(4500, radius)
        self.assertEquals([4500, 0], vector_to_target)

        gtap = GoToAbsolutePosition([0, 1500, 0])
        current_x = -4500
        current_y = 0
        vector_to_target, radius, angle = gtap.determine_vector_and_angle_towards_target(current_x, current_y)
        self.assertEquals(18, angle)
        self.assertEquals((4500 ** 2 + 1500 ** 2) ** 0.5, radius)
        self.assertEquals([4500, 1500], vector_to_target)

    def test_determine_rotation_for_angle(self):
        gtap = GoToAbsolutePosition([0, 0, 0])

        rotation = gtap.determine_rotation_from_to_angle(0, 0)
        self.assertEquals(0, rotation)

        rotation = gtap.determine_rotation_from_to_angle(0, 1)
        self.assertEquals(1, rotation)

        rotation = gtap.determine_rotation_from_to_angle(0, 10)
        self.assertEquals(10, rotation)

        rotation = gtap.determine_rotation_from_to_angle(0, 359)
        self.assertEquals(-1, rotation)

        rotation = gtap.determine_rotation_from_to_angle(0, 350)
        self.assertEquals(-10, rotation)

        rotation = gtap.determine_rotation_from_to_angle(0, 180)
        self.assertEquals(180, rotation)

        rotation = gtap.determine_rotation_from_to_angle(75, 75)
        self.assertEquals(0, rotation)

        rotation = gtap.determine_rotation_from_to_angle(75, 255)
        self.assertEquals(180, rotation)
