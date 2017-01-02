# -*- coding:utf-8 -*-

__author__ = 'daniel'
history_ = """
CalibrateVision
^^^^^^^^^^^^^^^

::moduleauthor:: Benjamin Scholtz, Judith Hartfill, Daniel Speck

History:
"""
from collections import OrderedDict
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.debug.test.export_log import ExportLog
from bitbots.modules.behaviour.body.actions.wait import Wait
from bitbots.util import get_config

config = get_config()


class CalibrateVision(AbstractDecisionModule):
    """
        Records data for vision calibration purposes
    """

    def __init__(self, _):
        super(CalibrateVision, self).__init__()
        self.exportLog = ExportLog("CalibrateVision.log")
        self.execCount = 0
        return

    def perform(self, connector, reevaluate=False):

        connector.walking_capsule().stop_walking()

        # If interrupt from throw comes here we want to wait until its over
        if connector.animation_capsule().is_animation_busy():
            return

        # record data
        goal_info_filtered = connector.filtered_vision_capsule().get_local_goal_filtered()
        goal_info_unfiltered = connector.raw_vision_capsule().get_goal_infos()

        # stop if no data is recorded
        if goal_info_filtered is None or goal_info_unfiltered is None:
            return

        # return if only one post is seen
        if len(goal_info_unfiltered) < 2:
            return

        # build log data set
        export_data = OrderedDict()

        self.execCount += 1
        export_data["execCount"] = self.execCount

        export_data["post1_u_raw"] = goal_info_unfiltered[0].u
        export_data["post1_v_raw"] = goal_info_unfiltered[0].v

        export_data["post2_u_raw"] = goal_info_unfiltered[1].u
        export_data["post2_v_raw"] = goal_info_unfiltered[1].v

        goalcenter_unfiltered_u = (goal_info_unfiltered[0].u + goal_info_unfiltered[1].u) / 2.0
        goalcenter_unfiltered_v = (goal_info_unfiltered[0].v + goal_info_unfiltered[0].v) / 2.0

        export_data["center_u_raw"] = goalcenter_unfiltered_u
        export_data["center_v_raw"] = goalcenter_unfiltered_v

        export_data["post1_u_filtered"] = goal_info_filtered.u_post1
        export_data["post1_v_filtered"] = goal_info_filtered.v_post1

        export_data["post2_u_filtered"] = goal_info_filtered.u_post2
        export_data["post2_v_filtered"] = goal_info_filtered.v_post2

        export_data["center_u_filtered"] = goal_info_filtered.u_center
        export_data["center_v_filtered"] = goal_info_filtered.v_center

        # add data set to buffer
        self.exportLog.addDataRecord(export_data)

        # write data
        self.exportLog.writeLog()

        return self.push(Wait)
