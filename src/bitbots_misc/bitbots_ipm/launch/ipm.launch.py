#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def ipm(
    sim: bool = False,
    full_image: bool = False,
    markers: bool = True,
    rviz: bool = False,
):
    """
    Parameters
    ----------
    sim : bool
        Whether the robot is running in simulation.
    full_image : bool
        Whether to project the full-size RGB image for debugging or showcasing.
    markers : bool
        Whether to publish markers for visualization of the detected objects in RViz.
    rviz : bool
        Whether to start RViz with the ipm configuration.
    """
    bl = BetterLaunch()

    bl.node(
        "ipm_image_node",
        "ipm",
        "ipm_line_mask",
        remaps={
            "camera_info": "/zed/zed_node/rgb/camera_info",
            "input": "/line_mask_in_image",
            "projected_point_cloud": "/line_mask_relative_pc",
        },
        params={
            "output_frame": "base_footprint",
            "scale": 0.2,
            "type": "mask",
            "use_distortion": True,
        },
        use_sim_time=sim,
    )

    bl.node(
        "soccer_ipm",
        "ipm",
        "soccer_ipm",
        remaps={"camera_info": "/zed/zed_node/rgb/camera_info"},
        param_files=bl.find("bitbots_ipm", "soccer_ipm.yaml", "config"),
        use_sim_time=sim,
    )

    if full_image:
        bl.node(
            "ipm_image_node",
            "ipm",
            "ipm_image",
            remaps={
                "camera_info": "/zed/zed_node/rgb/camera_info",
                "input": "/zed/zed_node/rgb/image_rect_color",
                "projected_point_cloud": "/projected_camera_image",
            },
            params={
                "output_frame": "base_footprint",
                "scale": 1.0,
                "type": "rgb_image",
                "use_distortion": True,
            },
            use_sim_time=sim,
        )

    if markers:
        bl.node(
            "soccer_vision_3d_rviz_markers",
            "visualizer",
            "soccer_vision_3d_rviz_marker_visualizer",
            remaps={
                "soccer_vision_3d/field_boundary": "/field_boundary_relative",
                "soccer_vision_3d/balls": "/balls_relative",
                "soccer_vision_3d/goalposts": "/goal_posts_relative",
                "soccer_vision_3d/robots": "/robots_relative",
                "soccer_vision_3d/obstacles": "/obstacles_relative",
                "soccer_vision_3d/markings": "/markings_relative",
            },
            # IMPORTANT: Ball diameter is ALSO defined in the soccer_ipm config file
            params={"ball_diameter": 0.153},
            use_sim_time=sim,
        )

    if rviz:
        bl.node(
            "rviz2",
            "rviz2",
            cmd_args=["-d", bl.find("bitbots_ipm", "ipm.rviz", "config")],
            log_level=None,
        )
