#!/usr/bin/env python3
import logging

from better_launch import BetterLaunch, launch_this


@launch_this
def aggregator():
    bl = BetterLaunch()
    bl.node(
        "diagnostic_aggregator",
        "aggregator_node",
        "",
        params="analyzers.yaml",
        log_level=logging.WARNING,
    )
