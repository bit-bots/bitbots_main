#!/usr/bin/env python3
import glob
import os

from bitbots_localization_handler import localization_dsd
from dynamic_stack_decider import DSD


def test_dsd_valid():
    # Create empty blackboard
    dummy_blackboard = object
    # Create DSD
    dsd = DSD(dummy_blackboard())

    # Register actions and decisions
    dsd.register_actions(localization_dsd.actions.__path__[0])
    dsd.register_decisions(localization_dsd.decisions.__path__[0])

    # Load all dsd files to check if they are valid\
    for dsd_file in glob.glob(os.path.join(localization_dsd.__path__[0], "*.dsd")):
        dsd.load_behavior(dsd_file)
