#!/usr/bin/env python3

import math
import yaml

import cv2
import numpy as np
from typing import Optional
from scipy import ndimage
from enum import Enum


class MapTypes(Enum):
    LINE = "line"
    POSTS = "posts"
    FIELD_BOUNDARY = "field_boundary"
    FIELD_FEATURES = "field_features"
    CORNERS = "corners"
    TCROSSINGS = "tcrossings"
    CROSSES = "crosses"


class MarkTypes(Enum):
    POINT = "point"
    CROSS = "cross"


class FieldFeatureStyles(Enum):
    EXACT = "exact"
    BLOB = "blob"


def drawCross(img, point, color, width=5, length=15):
    vertical_start = (point[0], point[1] - length)
    vertical_end = (point[0], point[1] + length)
    horizontal_start = (point[0] - length, point[1])
    horizontal_end = (point[0] + length, point[1])
    img = cv2.line(img, vertical_start, vertical_end, color, width)
    img = cv2.line(img, horizontal_start, horizontal_end, color, width)


def drawDistance(image, decay_factor):
    # Calc distances
    distance_map = ndimage.distance_transform_edt(255 - image)

    # Activation function that leads to a faster decay at the beginning
    # and a slower decay at the end and is parameterized by the decay factor
    if not math.isclose(decay_factor, 0):
        distance_map = np.exp(-distance_map * decay_factor * 0.1)
    else:
        distance_map = -distance_map

    # Scale to bounds of uint8 (0-255)
    return cv2.normalize(
        distance_map,
        None,
        alpha=0,
        beta=255,
        norm_type=cv2.NORM_MINMAX,
        dtype=cv2.CV_64F,
    ).astype(np.uint8)


def generate_map_image(parameters):
    target = MapTypes(parameters["map_type"])
    mark_type = MarkTypes(parameters["mark_type"])
    field_feature_style = FieldFeatureStyles(parameters["field_feature_style"])

    penalty_mark = parameters["penalty_mark"]
    center_point = parameters["center_point"]
    goal_back = parameters["goal_back"]  # Draw goal back area

    stroke_width = parameters["stroke_width"]
    field_length = parameters["field_length"]
    field_width = parameters["field_width"]
    goal_depth = parameters["goal_depth"]
    goal_width = parameters["goal_width"]
    goal_area_length = parameters["goal_area_length"]
    goal_area_width = parameters["goal_area_width"]
    penalty_mark_distance = parameters["penalty_mark_distance"]
    center_circle_diameter = parameters["center_circle_diameter"]
    border_strip_width = parameters["border_strip_width"]
    penalty_area_length = parameters["penalty_area_length"]
    penalty_area_width = parameters["penalty_area_width"]
    field_feature_size = parameters["field_feature_size"]
    distance_map = parameters["distance_map"]
    distance_decay = parameters["distance_decay"]
    invert = parameters["invert"]

    # Color of the lines and marks
    color = (255, 255, 255)  # white

    # Size of complete turf field (field with outside borders)
    image_size = (
        field_width + border_strip_width * 2,
        field_length + border_strip_width * 2,
        3,
    )

    # Calculate important points on the field
    field_outline_start = (border_strip_width, border_strip_width)
    field_outline_end = (
        field_length + border_strip_width,
        field_width + border_strip_width,
    )

    middle_line_start = (field_length // 2 + border_strip_width, border_strip_width)
    middle_line_end = (
        field_length // 2 + border_strip_width,
        field_width + border_strip_width,
    )

    middle_point = (
        field_length // 2 + border_strip_width,
        field_width // 2 + border_strip_width,
    )

    penalty_mark_left = (
        penalty_mark_distance + border_strip_width,
        field_width // 2 + border_strip_width,
    )
    penalty_mark_right = (
        image_size[1] - border_strip_width - penalty_mark_distance,
        field_width // 2 + border_strip_width,
    )

    goal_area_left_start = (
        border_strip_width,
        border_strip_width + field_width // 2 - goal_area_width // 2,
    )
    goal_area_left_end = (
        border_strip_width + goal_area_length,
        field_width // 2 + border_strip_width + goal_area_width // 2,
    )

    goal_area_right_start = (
        image_size[1] - goal_area_left_start[0],
        goal_area_left_start[1],
    )
    goal_area_right_end = (image_size[1] - goal_area_left_end[0], goal_area_left_end[1])

    penalty_area_left_start = (
        border_strip_width,
        border_strip_width + field_width // 2 - penalty_area_width // 2,
    )
    penalty_area_left_end = (
        border_strip_width + penalty_area_length,
        field_width // 2 + border_strip_width + penalty_area_width // 2,
    )

    penalty_area_right_start = (
        image_size[1] - penalty_area_left_start[0],
        penalty_area_left_start[1],
    )
    penalty_area_right_end = (
        image_size[1] - penalty_area_left_end[0],
        penalty_area_left_end[1],
    )

    goalpost_left_1 = (
        border_strip_width,
        border_strip_width + field_width // 2 + goal_width // 2,
    )
    goalpost_left_2 = (
        border_strip_width,
        border_strip_width + field_width // 2 - goal_width // 2,
    )

    goalpost_right_1 = (image_size[1] - goalpost_left_1[0], goalpost_left_1[1])
    goalpost_right_2 = (image_size[1] - goalpost_left_2[0], goalpost_left_2[1])

    goal_back_corner_left_1 = (goalpost_left_1[0] - goal_depth, goalpost_left_1[1])
    goal_back_corner_left_2 = (goalpost_left_2[0] - goal_depth, goalpost_left_2[1])

    goal_back_corner_right_1 = (goalpost_right_1[0] + goal_depth, goalpost_right_1[1])
    goal_back_corner_right_2 = (goalpost_right_2[0] + goal_depth, goalpost_right_2[1])

    # Create black image in the correct size
    img = np.zeros(image_size, np.uint8)

    # Check which map type we want to generate
    if target == MapTypes.LINE:
        # Draw outline
        img = cv2.rectangle(
            img, field_outline_start, field_outline_end, color, stroke_width
        )

        # Draw middle line
        img = cv2.line(img, middle_line_start, middle_line_end, color, stroke_width)

        # Draw center circle
        img = cv2.circle(
            img, middle_point, center_circle_diameter // 2, color, stroke_width
        )

        # Draw center mark (point or cross)
        if center_point:
            if mark_type == MarkTypes.POINT:
                img = cv2.circle(img, middle_point, stroke_width * 2, color, -1)
            elif mark_type == MarkTypes.CROSS:
                drawCross(img, middle_point, color, stroke_width)
            else:
                raise NotImplementedError("Mark type not implemented")

        # Draw penalty marks (point or cross)
        if penalty_mark:
            if mark_type == MarkTypes.POINT:
                img = cv2.circle(img, penalty_mark_left, stroke_width * 2, color, -1)
                img = cv2.circle(img, penalty_mark_right, stroke_width * 2, color, -1)
            elif mark_type == MarkTypes.CROSS:
                drawCross(img, penalty_mark_left, color, stroke_width)
                drawCross(img, penalty_mark_right, color, stroke_width)
            else:
                raise NotImplementedError("Mark type not implemented")

        # Draw goal area
        img = cv2.rectangle(
            img, goal_area_left_start, goal_area_left_end, color, stroke_width
        )
        img = cv2.rectangle(
            img, goal_area_right_start, goal_area_right_end, color, stroke_width
        )

        # Draw penalty area
        img = cv2.rectangle(
            img, penalty_area_left_start, penalty_area_left_end, color, stroke_width
        )
        img = cv2.rectangle(
            img, penalty_area_right_start, penalty_area_right_end, color, stroke_width
        )

        # Draw goal back area
        if goal_back:
            img = cv2.rectangle(
                img, goalpost_left_1, goal_back_corner_left_2, color, stroke_width
            )
            img = cv2.rectangle(
                img, goalpost_right_1, goal_back_corner_right_2, color, stroke_width
            )

    if target == MapTypes.POSTS:
        # Draw goalposts
        img = cv2.circle(img, goalpost_left_1, stroke_width * 2, color, -1)
        img = cv2.circle(img, goalpost_left_2, stroke_width * 2, color, -1)
        img = cv2.circle(img, goalpost_right_1, stroke_width * 2, color, -1)
        img = cv2.circle(img, goalpost_right_2, stroke_width * 2, color, -1)

    if target == MapTypes.FIELD_BOUNDARY:
        # We need a larger image for this as we draw outside the field
        img = np.zeros((image_size[0] + 200, image_size[1] + 200), np.uint8)

        # Draw fieldboundary
        img = cv2.rectangle(
            img,
            (100, 100),
            (image_size[1] + 100, image_size[0] + 100),
            color,
            stroke_width,
        )

    if (
        target in [MapTypes.CORNERS, MapTypes.FIELD_FEATURES]
        and field_feature_style == FieldFeatureStyles.EXACT
    ):
        # draw outline corners
        # top left
        img = cv2.line(
            img,
            field_outline_start,
            (field_outline_start[0], field_outline_start[1] + field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            field_outline_start,
            (field_outline_start[0] + field_feature_size, field_outline_start[1]),
            color,
            stroke_width,
        )

        # bottom left
        img = cv2.line(
            img,
            (field_outline_start[0], field_outline_end[1]),
            (field_outline_start[0], field_outline_end[1] - field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            (field_outline_start[0], field_outline_end[1]),
            (field_outline_start[0] + field_feature_size, field_outline_end[1]),
            color,
            stroke_width,
        )

        # top right
        img = cv2.line(
            img,
            (field_outline_end[0], field_outline_start[1]),
            (field_outline_end[0], field_outline_start[1] + field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            (field_outline_end[0], field_outline_start[1]),
            (field_outline_end[0] - field_feature_size, field_outline_start[1]),
            color,
            stroke_width,
        )

        # bottom right
        img = cv2.line(
            img,
            field_outline_end,
            (field_outline_end[0], field_outline_end[1] - field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            field_outline_end,
            (field_outline_end[0] - field_feature_size, field_outline_end[1]),
            color,
            stroke_width,
        )

        # draw left goal area corners
        # top
        img = cv2.line(
            img,
            (goal_area_left_end[0], goal_area_left_start[1]),
            (goal_area_left_end[0], goal_area_left_start[1] + field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            (goal_area_left_end[0], goal_area_left_start[1]),
            (goal_area_left_end[0] - field_feature_size, goal_area_left_start[1]),
            color,
            stroke_width,
        )

        # bottom

        img = cv2.line(
            img,
            goal_area_left_end,
            (goal_area_left_end[0], goal_area_left_end[1] - field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            goal_area_left_end,
            (goal_area_left_end[0] - field_feature_size, goal_area_left_end[1]),
            color,
            stroke_width,
        )

        # draw right goal aera corners

        # top

        img = cv2.line(
            img,
            (goal_area_right_end[0], goal_area_right_start[1]),
            (goal_area_right_end[0], goal_area_right_start[1] + field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            (goal_area_right_end[0], goal_area_right_start[1]),
            (goal_area_right_end[0] + field_feature_size, goal_area_right_start[1]),
            color,
            stroke_width,
        )

        # bottom

        img = cv2.line(
            img,
            goal_area_right_end,
            (goal_area_right_end[0], goal_area_right_end[1] - field_feature_size),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            goal_area_right_end,
            (goal_area_right_end[0] + field_feature_size, goal_area_right_end[1]),
            color,
            stroke_width,
        )

    if (
        target in [MapTypes.CORNERS, MapTypes.FIELD_FEATURES]
        and field_feature_style == FieldFeatureStyles.BLOB
    ):
        # field corners
        img = cv2.circle(img, field_outline_start, field_feature_size, color, -1)
        img = cv2.circle(
            img,
            (field_outline_start[0], field_outline_end[1]),
            field_feature_size,
            color,
            -1,
        )
        img = cv2.circle(
            img,
            (field_outline_end[0], field_outline_start[1]),
            field_feature_size,
            color,
            -1,
        )
        img = cv2.circle(img, field_outline_end, field_feature_size, color, -1)

        # goal area corners
        img = cv2.circle(
            img,
            (goal_area_left_end[0], goal_area_left_start[1]),
            field_feature_size,
            color,
            -1,
        )
        img = cv2.circle(img, goal_area_left_end, field_feature_size, color, -1)
        img = cv2.circle(
            img,
            (goal_area_right_end[0], goal_area_right_start[1]),
            field_feature_size,
            color,
            -1,
        )
        img = cv2.circle(img, goal_area_right_end, field_feature_size, color, -1)

    if (
        target in [MapTypes.TCROSSINGS, MapTypes.FIELD_FEATURES]
        and field_feature_style == FieldFeatureStyles.EXACT
    ):
        # draw left goal area
        # top
        img = cv2.line(
            img,
            (
                goal_area_left_start[0],
                goal_area_left_start[1] - field_feature_size // 2,
            ),
            (
                goal_area_left_start[0],
                goal_area_left_start[1] + field_feature_size // 2,
            ),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            goal_area_left_start,
            (goal_area_left_start[0] + field_feature_size, goal_area_left_start[1]),
            color,
            stroke_width,
        )

        # bottom
        img = cv2.line(
            img,
            (goal_area_left_start[0], goal_area_left_end[1] - field_feature_size // 2),
            (goal_area_left_start[0], goal_area_left_end[1] + field_feature_size // 2),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            (goal_area_left_start[0], goal_area_left_end[1]),
            (goal_area_left_start[0] + field_feature_size, goal_area_left_end[1]),
            color,
            stroke_width,
        )
        # draw right goal area

        # top
        img = cv2.line(
            img,
            (
                goal_area_right_start[0],
                goal_area_right_start[1] - field_feature_size // 2,
            ),
            (
                goal_area_right_start[0],
                goal_area_right_start[1] + field_feature_size // 2,
            ),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            goal_area_right_start,
            (goal_area_right_start[0] - field_feature_size, goal_area_right_start[1]),
            color,
            stroke_width,
        )

        # bottom
        img = cv2.line(
            img,
            (
                goal_area_right_start[0],
                goal_area_right_end[1] - field_feature_size // 2,
            ),
            (
                goal_area_right_start[0],
                goal_area_right_end[1] + field_feature_size // 2,
            ),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            (goal_area_right_start[0], goal_area_right_end[1]),
            (goal_area_right_start[0] - field_feature_size, goal_area_right_end[1]),
            color,
            stroke_width,
        )

        # draw center line to side line t crossings
        # top
        img = cv2.line(
            img,
            (middle_line_start[0] - field_feature_size // 2, middle_line_start[1]),
            (middle_line_start[0] + field_feature_size // 2, middle_line_start[1]),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            middle_line_start,
            (middle_line_start[0], middle_line_start[1] + field_feature_size),
            color,
            stroke_width,
        )

        # bottom
        img = cv2.line(
            img,
            (middle_line_end[0] - field_feature_size // 2, middle_line_end[1]),
            (middle_line_end[0] + field_feature_size // 2, middle_line_end[1]),
            color,
            stroke_width,
        )
        img = cv2.line(
            img,
            middle_line_end,
            (middle_line_end[0], middle_line_end[1] - field_feature_size),
            color,
            stroke_width,
        )

    if (
        target in [MapTypes.TCROSSINGS, MapTypes.FIELD_FEATURES]
        and field_feature_style == FieldFeatureStyles.BLOB
    ):
        # draw blobs for goal areas
        img = cv2.circle(img, goal_area_left_start, field_feature_size, color, -1)
        img = cv2.circle(
            img,
            (goal_area_left_start[0], goal_area_left_end[1]),
            field_feature_size,
            color,
            -1,
        )
        img = cv2.circle(img, goal_area_right_start, field_feature_size, color, -1)
        img = cv2.circle(
            img,
            (goal_area_right_start[0], goal_area_right_end[1]),
            field_feature_size,
            color,
            -1,
        )

        # middle line
        img = cv2.circle(img, middle_line_start, field_feature_size, color, -1)
        img = cv2.circle(img, middle_line_end, field_feature_size, color, -1)

    if (
        target in [MapTypes.CROSSES, MapTypes.FIELD_FEATURES]
        and field_feature_style == FieldFeatureStyles.EXACT
    ):
        # penalty marks
        if penalty_mark:
            drawCross(
                img, penalty_mark_left, color, stroke_width, field_feature_size // 2
            )
            drawCross(
                img, penalty_mark_right, color, stroke_width, field_feature_size // 2
            )

        # middle point
        if center_point:
            drawCross(img, middle_point, color, stroke_width, field_feature_size // 2)

        # center circle middle line crossings
        drawCross(
            img,
            (middle_point[0], middle_point[1] - center_circle_diameter),
            color,
            stroke_width,
            field_feature_size // 2,
        )
        drawCross(
            img,
            (middle_point[0], middle_point[1] + center_circle_diameter),
            color,
            stroke_width,
            field_feature_size // 2,
        )

    if (
        target in [MapTypes.CROSSES, MapTypes.FIELD_FEATURES]
        and field_feature_style == FieldFeatureStyles.BLOB
    ):
        # penalty marks
        if penalty_mark:
            img = cv2.circle(img, penalty_mark_left, field_feature_size, color, -1)
            img = cv2.circle(img, penalty_mark_right, field_feature_size, color, -1)

        # middle point
        if center_point:
            img = cv2.circle(img, middle_point, field_feature_size, color, -1)

        # center circle middle line crossings
        img = cv2.circle(
            img,
            (middle_point[0], middle_point[1] - center_circle_diameter),
            field_feature_size,
            color,
            -1,
        )
        img = cv2.circle(
            img,
            (middle_point[0], middle_point[1] + center_circle_diameter),
            field_feature_size,
            color,
            -1,
        )

    # Create distance map
    if distance_map:
        img = drawDistance(img, distance_decay)

    # Invert
    if invert:
        img = 255 - img

    return img


def generate_metadata(parameters: dict, image_name: str) -> dict:
    # Get the field dimensions in cm
    field_dimensions = np.array(
        [parameters["field_length"], parameters["field_width"], 0]
    )
    # Add the border strip
    field_dimensions[:2] += 2 * parameters["border_strip_width"]
    # Get the origin
    origin = -field_dimensions / 2
    # Convert to meters
    origin /= 100

    # Generate the metadata
    return {
        "image": image_name,
        "resolution": 0.01,
        "origin": origin.tolist(),
        "occupied_thresh": 0.99,
        "free_thresh": 0.196,
        "negate": int(parameters["invert"]),
    }


def load_config_file(file) -> Optional[dict]:
    # Load the parameters from the file
    config_file = yaml.load(file, Loader=yaml.FullLoader)
    # Check if the file is valid (has the correct fields)
    if (
        "header" in config_file
        and "type" in config_file["header"]
        and "version" in config_file["header"]
        and "parameters" in config_file
        and config_file["header"]["version"] == "1.0"
        and config_file["header"]["type"] == "map_generator_config"
    ):
        return config_file["parameters"]
