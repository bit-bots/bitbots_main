#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def education():
    bl = BetterLaunch()

    bl.include("rosbridge_server", "rosbridge_websocket_launch.xml")

    bl.node(
        "web_video_server",
        "web_video_server",
        "web_video_server_education",
        params={"port": 8081},
    )

    bl.node("bitbots_education", "webserver", "education_webserver")

    bl.process(
        ["ros2", "param", "set", "bitbots_vision", "component_debug_image_active", "true", "--timeout", "20000"],
        name="education_vision_debug_activator",
        output="screen",
    )
