import time

from stackmachine.abstract_decision_module import AbstractDecisionModule

from body.actions.plain_walk_action import PlainWalkAction
from body.actions.throw import RIGHT, LEFT
from stackmachine.model.capsules.walking_capsule import WalkingCapsule


class TurnAfterThrow(AbstractDecisionModule):
    def __init__(self, _):
        super(TurnAfterThrow, self).__init__()
        self.toggle_relocate_turn = config["Behaviour"]["Toggles"]["Goalie"]["relocateTurn"]
        self.anim_goalie_walkready = config["animations"]["motion"]["goalie-walkready"]
        self.accel_lst = []
        self.starttime = time.time()
        self.vote_accel_z = False

    def perform(self, connector, reevaluate=False):

        richtung = connector.blackboard_capsule().get_throw_direction()

        # we need to check if we fell down after getting up. controlable is true for a short time in between
        # Positiv value if we stand +128 z
        self.vote_accel_z = self.vote_accel_z_method(connector.get_ipc().get_accel())  # todo test this

        if connector.get_ipc().controlable:  # and self.vote_accel_z:
            if richtung == RIGHT:
                # todo make it more dynamic (no hard coded turn times) We should have the angle
                connector.blackboard_capsule().delete_was_thrown()
                return self.push(PlainWalkAction, [
                    [WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_LEFT, WalkingCapsule.ZERO, 4.5]])
            if richtung == LEFT:
                connector.blackboard_capsule().delete_was_thrown()
                return self.push(PlainWalkAction, [
                    [WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_RIGHT, WalkingCapsule.ZERO, 4.5]])

    def vote_accel_z_method(self, accel):

        if len(self.accel_lst) > 30:
            self.accel_lst = self.accel_lst[1:]

        self.accel_lst.append(accel.z)

        number_valid = sum(100 < v < 170 for v in self.accel_lst)

        return number_valid > 28
