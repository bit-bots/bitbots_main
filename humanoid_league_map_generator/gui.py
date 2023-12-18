import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from PIL import Image, ImageTk
import cv2
import math
import numpy as np
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


class MapGeneratorParamInput(tk.Frame):
    def __init__(self, parent, update_hook: callable, parameter_definitions: dict[str, dict]):
        tk.Frame.__init__(self, parent)

        # Keep track of parameter definitions, GUI elements and the input values
        self.parameter_definitions = parameter_definitions
        self.parameter_ui_elements: dict[str, dict[str, ttk.Widget]] = {}
        self.parameter_values: dict[str, tk.Variable] = {}

        # Generate GUI elements for all parameters                      
        for parameter_name, parameter_definition in parameter_definitions.items():
            # Create GUI elements
            label = ttk.Label(self, text=parameter_definition["label"])
            if parameter_definition["type"] == bool:
                variable = tk.BooleanVar(value=parameter_definition["default"])
                ui_element = ttk.Checkbutton(self, command=update_hook, variable=variable)
            elif parameter_definition["type"] == int:
                variable = tk.IntVar(value=parameter_definition["default"])
                ui_element = ttk.Entry(self, textvariable=variable)
                ui_element.bind("<KeyRelease>", update_hook)
            elif parameter_definition["type"] == float:
                variable = tk.DoubleVar(value=parameter_definition["default"])
                ui_element = ttk.Entry(self, textvariable=variable)
                ui_element.bind("<KeyRelease>", update_hook)
            elif issubclass(parameter_definition["type"], Enum):
                variable = tk.StringVar(value=parameter_definition["default"].value)
                values = [enum.value for enum in parameter_definition["type"]]
                ui_element = ttk.Combobox(self, values=values, textvariable=variable)
                ui_element.bind("<<ComboboxSelected>>", update_hook)
            else:
                raise NotImplementedError("Parameter type not implemented")
        
            self.parameter_values[parameter_name] = variable

            # Add to dict
            self.parameter_ui_elements[parameter_name] = {
                "label": label,
                "ui_element": ui_element
            }

        # Create layout
        for i, parameter_name in enumerate(parameter_definitions.keys()):
            self.parameter_ui_elements[parameter_name]["label"].grid(row=i, column=0, sticky="e")
            self.parameter_ui_elements[parameter_name]["ui_element"].grid(row=i, column=1, sticky="w")
    
    def get_parameters(self):
        return {parameter_name: parameter_value.get() for parameter_name, parameter_value in self.parameter_values.items()}
    
    def get_parameter(self, parameter_name):
        return self.parameter_values[parameter_name].get()



class MapGeneratorGUI:
    def __init__(self, root: tk.Tk):
        # Set ttk theme
        s = ttk.Style()
        s.theme_use('clam')

        # Set window title and size
        self.root = root
        self.root.title("Map Generator GUI")
        self.root.resizable(False, False)

        # Create GUI elements

        # Output folder
        self.output_label = ttk.Label(self.root, text="Output Folder:")
        self.output_folder = tk.StringVar()
        self.output_entry = ttk.Entry(self.root, textvariable=self.output_folder)
        self.output_button = ttk.Button(self.root, text="Browse", command=self.browse_output_folder)

        # Parameter Input
        self.parameter_input = MapGeneratorParamInput(self.root, self.update_map, {
            "map_type": {
                "type": MapTypes,
                "default": MapTypes.LINE,
                "label": "Map Type",
                "tooltip": "Type of the map we want to generate"
            },
            "penalty_mark": {
                "type": bool,
                "default": True,
                "label": "Penalty Mark",
                "tooltip": "Draw penalty mark"
            },
            "center_point": {
                "type": bool,
                "default": True,
                "label": "Center Point",
                "tooltip": "Draw center point"
            },
            "goal_back": {
                "type": bool,
                "default": True,
                "label": "Goal Back",
                "tooltip": "Draw goal back area"
            },
            "stroke_width": {
                "type": int,
                "default": 5,
                "label": "Stoke Width",
                "tooltip": "Width of the shapes we draw"
            },
            "field_length": {
                "type": int,
                "default": 900,
                "label": "Field Length",
                "tooltip": "Length of the field"
            },
            "field_width": {
                "type": int,
                "default": 600,
                "label": "Field Width",
                "tooltip": "Width of the field"
            },
            "goal_depth": {
                "type": int,
                "default": 60,
                "label": "Goal Depth",
                "tooltip": "Depth of the goal"
            },
            "goal_width": {
                "type": int,
                "default": 260,
                "label": "Goal Width",
                "tooltip": "Width of the goal"
            },
            "goal_area_length": {
                "type": int,
                "default": 100,
                "label": "Goal Area Length",
                "tooltip": "Length of the goal area"
            },
            "goal_area_width": {
                "type": int,
                "default": 300,
                "label": "Goal Area Width",
                "tooltip": "Width of the goal area"
            },
            "penalty_mark_distance": {
                "type": int,
                "default": 150,
                "label": "Penalty Mark Distance",
                "tooltip": "Distance of the penalty mark from the goal"
            },
            "center_circle_diameter": {
                "type": int,
                "default": 150,
                "label": "Center Circle Diameter",
                "tooltip": "Diameter of the center circle"
            },
            "border_strip_width": {
                "type": int,
                "default": 100,
                "label": "Border Strip Width",
                "tooltip": "Width of the border strip"
            },
            "penalty_area_length": {
                "type": int,
                "default": 200,
                "label": "Penalty Area Length",
                "tooltip": "Length of the penalty area"
            },
            "penalty_area_width": {
                "type": int,
                "default": 500,
                "label": "Penalty Area Width",
                "tooltip": "Width of the penalty area"
            },
            "field_feature_size": {
                "type": int,
                "default": 30,
                "label": "Field Feature Size",
                "tooltip": "Size of the field features"
            },
            "mark_type": {
                "type": MarkTypes,
                "default": MarkTypes.CROSS,
                "label": "Mark Type",
                "tooltip": "Type of the marks"
            },
            "field_feature_style": {
                "type": FieldFeatureStyles,
                "default": FieldFeatureStyles.EXACT,
                "label": "Field Feature Style",
                "tooltip": "Style of the field features"
            },
            "blur_factor": {
                "type": float,
                "default": 0.0,
                "label": "Blur Factor",
                "tooltip": "Blur factor"
            },
            "invert": {
                "type": bool,
                "default": False,
                "label": "Invert",
                "tooltip": "Invert the image"
            }
        })

        # Generate Map Button
        self.generate_button = ttk.Button(self.root, text="Generate Map", command=self.update_map)

        # Canvas to display the generated map
        self.canvas = tk.Canvas(self.root, width=800, height=600)

        # Layout


        # Output folder
        self.output_label.grid(row=0, column=0, sticky="e")
        self.output_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.output_button.grid(row=0, column=2, padx=5, pady=5)

        # Parameter input and generate button
        self.parameter_input.grid(row=1, column=0, columnspan=3, pady=10)
        self.generate_button.grid(row=2, column=0, columnspan=3, pady=10)

        # Preview
        self.canvas.grid(row=0, column=3, rowspan=3)

        # Color in which we want to draw the lines
        self.primary_color = (255, 255, 255)  # white

        # Render initial map
        self.root.update()  # We need to call update() first 
        self.update_map()
        

    def browse_output_folder(self):
        folder = filedialog.askdirectory()
        if folder:
            self.output_folder.set(folder)

    def update_map(self, *args):
        # Generate and display the map on the canvas
        generated_map = self.generate_map_image()
        self.display_map(generated_map)

    def drawCross(self, img, point, color, width=5, length=15):
        half_width = width // 2 + width % 2
        vertical_start = (point[0] - half_width, point[1] - length)
        vertical_end = (point[0] + half_width, point[1] + length)
        horizontal_start = (point[0] - length, point[1] - half_width)
        horizontal_end = (point[0] + length, point[1] + half_width)
        img = cv2.rectangle(img, vertical_start, vertical_end, color, -1)
        img = cv2.rectangle(img, horizontal_start, horizontal_end, color, -1)

    def blurDistance(self, image, blur_factor):
        if blur_factor <= 0:  # Skip blur
            return 255 - image

        # Calc distances
        distance_map = 255 - ndimage.morphology.distance_transform_edt(255 - image)

        # Maximum field distance
        maximum_size = math.sqrt(image.shape[0] ** 2 + image.shape[1] ** 2)

        # Get the value to a value from 0 to 1
        distance_map = (distance_map / maximum_size)

        # Magic value please change it
        beta = (1 - blur_factor) * 10

        # Activation function
        distance_map = distance_map ** (2 * beta)

        # Scale up
        distance_map = cv2.normalize(distance_map, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        # To img
        out_img = 100 - distance_map.astype(np.uint8)
        return out_img

    def generate_map_image(self):
        target = MapTypes(self.parameter_input.get_parameters()["map_type"])
        mark_type = MarkTypes(self.parameter_input.get_parameters()["mark_type"])
        field_feature_style = FieldFeatureStyles(self.parameter_input.get_parameters()["field_feature_style"])

        penalty_mark = self.parameter_input.get_parameters()["penalty_mark"]
        center_point = self.parameter_input.get_parameters()["center_point"]
        goal_back = self.parameter_input.get_parameters()["goal_back"]  # Draw goal back area

        stroke_width = self.parameter_input.get_parameters()["stroke_width"]
        field_length = self.parameter_input.get_parameters()["field_length"]
        field_width = self.parameter_input.get_parameters()["field_width"]
        goal_depth = self.parameter_input.get_parameters()["goal_depth"]
        goal_width = self.parameter_input.get_parameters()["goal_width"]
        goal_area_length = self.parameter_input.get_parameters()["goal_area_length"]
        goal_area_width = self.parameter_input.get_parameters()["goal_area_width"]
        penalty_mark_distance = self.parameter_input.get_parameters()["penalty_mark_distance"]
        center_circle_diameter = self.parameter_input.get_parameters()["center_circle_diameter"]
        border_strip_width = self.parameter_input.get_parameters()["border_strip_width"]
        penalty_area_length = self.parameter_input.get_parameters()["penalty_area_length"]
        penalty_area_width = self.parameter_input.get_parameters()["penalty_area_width"]
        field_feature_size = self.parameter_input.get_parameters()["field_feature_size"]
        blur_factor = self.parameter_input.get_parameters()["blur_factor"]
        invert = self.parameter_input.get_parameters()["invert"]
        color = self.primary_color

        # Size of complete turf field (field with outside borders)
        image_size = (field_width + border_strip_width * 2,
                    field_length + border_strip_width * 2,
                    3)

        # Calculate important points on the field
        field_outline_start = (border_strip_width, border_strip_width)
        field_outline_end = (field_length + border_strip_width,
                            field_width + border_strip_width)

        middle_line_start = (field_length // 2 + border_strip_width,
                            border_strip_width)
        middle_line_end = (field_length // 2 + border_strip_width,
                        field_width + border_strip_width)

        middle_point = (field_length // 2 + border_strip_width,
                        field_width // 2 + border_strip_width)

        penalty_mark_left = (penalty_mark_distance + border_strip_width,
                            field_width // 2 + border_strip_width)
        penalty_mark_right = (image_size[1] - border_strip_width - penalty_mark_distance,
                            field_width // 2 + border_strip_width)

        goal_area_left_start = (border_strip_width,
                                border_strip_width + field_width // 2 - goal_area_width // 2)
        goal_area_left_end = (border_strip_width + goal_area_length,
                            field_width // 2 + border_strip_width + goal_area_width // 2)

        goal_area_right_start = (image_size[1] - goal_area_left_start[0],
                                goal_area_left_start[1])
        goal_area_right_end = (image_size[1] - goal_area_left_end[0],
                            goal_area_left_end[1])

        penalty_area_left_start = (border_strip_width,
                                border_strip_width + field_width // 2 - penalty_area_width // 2)
        penalty_area_left_end = (border_strip_width + penalty_area_length,
                                field_width // 2 + border_strip_width + penalty_area_width // 2)

        penalty_area_right_start = (image_size[1] - penalty_area_left_start[0],
                                    penalty_area_left_start[1])
        penalty_area_right_end = (image_size[1] - penalty_area_left_end[0],
                                penalty_area_left_end[1])

        goalpost_left_1 = (border_strip_width,
                        border_strip_width + field_width // 2 + goal_width // 2)
        goalpost_left_2 = (border_strip_width,
                        border_strip_width + field_width // 2 - goal_width // 2)

        goalpost_right_1 = (image_size[1] - goalpost_left_1[0], goalpost_left_1[1])
        goalpost_right_2 = (image_size[1] - goalpost_left_2[0], goalpost_left_2[1])

        goal_back_corner_left_1 = (goalpost_left_1[0] - goal_depth,
                                goalpost_left_1[1])
        goal_back_corner_left_2 = (goalpost_left_2[0] - goal_depth,
                                goalpost_left_2[1])

        goal_back_corner_right_1 = (goalpost_right_1[0] + goal_depth,
                                    goalpost_right_1[1])
        goal_back_corner_right_2 = (goalpost_right_2[0] + goal_depth,
                                    goalpost_right_2[1])


        
        # Create black image in the correct size
        img = np.zeros(image_size, np.uint8)

        # Check which map type we want to generate
        if target == MapTypes.LINE:

            # Draw outline
            img = cv2.rectangle(img, field_outline_start, field_outline_end, color, stroke_width)

            # Draw middle line
            img = cv2.line(img, middle_line_start, middle_line_end, color, stroke_width)

            # Draw center circle
            img = cv2.circle(img, middle_point, center_circle_diameter // 2, color, stroke_width)

            # Draw center mark (point or cross)
            if center_point:
                if mark_type == MarkTypes.POINT:
                    img = cv2.circle(img, middle_point, stroke_width * 2, color, -1)
                elif mark_type == MarkTypes.CROSS:
                    self.drawCross(img, middle_point, color, stroke_width)
                else:
                    raise NotImplementedError("Mark type not implemented")

            # Draw penalty marks (point or cross)
            if penalty_mark:
                if mark_type == MarkTypes.POINT:
                    img = cv2.circle(img, penalty_mark_left, stroke_width * 2, color, -1)
                    img = cv2.circle(img, penalty_mark_right, stroke_width * 2, color, -1)
                elif mark_type == MarkTypes.CROSS:
                    self.drawCross(img, penalty_mark_left, color, stroke_width)
                    self.drawCross(img, penalty_mark_right, color, stroke_width)
                else:
                    raise NotImplementedError("Mark type not implemented")

            # Draw goal area
            img = cv2.rectangle(img, goal_area_left_start, goal_area_left_end, color, stroke_width)
            img = cv2.rectangle(img, goal_area_right_start, goal_area_right_end, color, stroke_width)

            # Draw penalty area
            img = cv2.rectangle(img, penalty_area_left_start, penalty_area_left_end, color, stroke_width)
            img = cv2.rectangle(img, penalty_area_right_start, penalty_area_right_end, color, stroke_width)

            # Draw goal back area
            if goal_back:
                img = cv2.rectangle(img, goalpost_left_1, goal_back_corner_left_2, color, stroke_width)
                img = cv2.rectangle(img, goalpost_right_1, goal_back_corner_right_2, color, stroke_width)

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
                stroke_width)
            
        if target in [MapTypes.CORNERS, MapTypes.FIELD_FEATURES] and field_feature_style == FieldFeatureStyles.EXACT:
            # draw outline corners
            # top left
            img = cv2.line(img, field_outline_start,
                                (field_outline_start[0], field_outline_start[1] + field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, field_outline_start,
                                (field_outline_start[0] + field_feature_size, field_outline_start[1]),
                                color, stroke_width)

            # bottom left
            img = cv2.line(img, (field_outline_start[0], field_outline_end[1]),
                                (field_outline_start[0], field_outline_end[1] - field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, (field_outline_start[0], field_outline_end[1]),
                                (field_outline_start[0] + field_feature_size, field_outline_end[1]),
                                color, stroke_width)

            # top right
            img = cv2.line(img, (field_outline_end[0], field_outline_start[1]),
                                (field_outline_end[0], field_outline_start[1] + field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, (field_outline_end[0], field_outline_start[1]),
                                (field_outline_end[0] - field_feature_size, field_outline_start[1]),
                                color, stroke_width)

            # bottom right
            img = cv2.line(img, field_outline_end,
                                (field_outline_end[0], field_outline_end[1] - field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, field_outline_end,
                                (field_outline_end[0] - field_feature_size, field_outline_end[1]),
                                color, stroke_width)

            # draw left goal area corners
            # top
            img = cv2.line(img, (goal_area_left_end[0], goal_area_left_start[1]),
                                (goal_area_left_end[0], goal_area_left_start[1] + field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, (goal_area_left_end[0], goal_area_left_start[1]),
                                (goal_area_left_end[0] - field_feature_size, goal_area_left_start[1]),
                                color, stroke_width)

            # bottom

            img = cv2.line(img, goal_area_left_end,
                                (goal_area_left_end[0], goal_area_left_end[1] - field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, goal_area_left_end,
                                (goal_area_left_end[0] - field_feature_size, goal_area_left_end[1]),
                                color, stroke_width)

            # draw right goal aera corners

            # top

            img = cv2.line(img, (goal_area_right_end[0], goal_area_right_start[1]),
                                (goal_area_right_end[0], goal_area_right_start[1] + field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, (goal_area_right_end[0], goal_area_right_start[1]),
                                (goal_area_right_end[0] + field_feature_size, goal_area_right_start[1]),
                                color, stroke_width)

            # bottom

            img = cv2.line(img, goal_area_right_end,
                                (goal_area_right_end[0], goal_area_right_end[1] - field_feature_size),
                                color, stroke_width)
            img = cv2.line(img, goal_area_right_end,
                                (goal_area_right_end[0] + field_feature_size, goal_area_right_end[1]),
                                color, stroke_width)

        if target in [MapTypes.CORNERS, MapTypes.FIELD_FEATURES] and field_feature_style == FieldFeatureStyles.BLOB:
            # field corners
            img = cv2.circle(img, field_outline_start, field_feature_size, color, -1)
            img = cv2.circle(img, (field_outline_start[0], field_outline_end[1]), field_feature_size, color, -1)
            img = cv2.circle(img, (field_outline_end[0], field_outline_start[1]), field_feature_size, color, -1)
            img = cv2.circle(img, field_outline_end, field_feature_size, color, -1)

            # goal area corners
            img = cv2.circle(img, (goal_area_left_end[0], goal_area_left_start[1]), field_feature_size, color, -1)
            img = cv2.circle(img, goal_area_left_end, field_feature_size, color, -1)
            img = cv2.circle(img, (goal_area_right_end[0], goal_area_right_start[1]), field_feature_size, color, -1)
            img = cv2.circle(img, goal_area_right_end, field_feature_size, color, -1)


        if target in [MapTypes.TCROSSINGS, MapTypes.FIELD_FEATURES] and field_feature_style == FieldFeatureStyles.EXACT:
            # draw left goal area
            # top
            img = cv2.line(img, (goal_area_left_start[0], goal_area_left_start[1] - field_feature_size // 2),
                                    (goal_area_left_start[0], goal_area_left_start[1] + field_feature_size // 2),
                                    color, stroke_width)
            img = cv2.line(img, goal_area_left_start,
                                    (goal_area_left_start[0] + field_feature_size, goal_area_left_start[1]),
                                    color, stroke_width)

            # bottom
            img = cv2.line(img, (goal_area_left_start[0], goal_area_left_end[1] - field_feature_size // 2),
                                    (goal_area_left_start[0], goal_area_left_end[1] + field_feature_size // 2),
                                    color, stroke_width)
            img = cv2.line(img, (goal_area_left_start[0], goal_area_left_end[1]),
                                    (goal_area_left_start[0] + field_feature_size, goal_area_left_end[1]),
                                    color, stroke_width)
            # draw right goal area

            # top
            img = cv2.line(img, (goal_area_right_start[0], goal_area_right_start[1] - field_feature_size // 2),
                                    (goal_area_right_start[0], goal_area_right_start[1] + field_feature_size // 2),
                                    color, stroke_width)
            img = cv2.line(img, goal_area_right_start,
                                    (goal_area_right_start[0] - field_feature_size, goal_area_right_start[1]),
                                    color, stroke_width)
            
            # bottom
            img = cv2.line(img, (goal_area_right_start[0], goal_area_right_end[1] - field_feature_size // 2),
                                    (goal_area_right_start[0], goal_area_right_end[1] + field_feature_size // 2),
                                    color, stroke_width)
            img = cv2.line(img, (goal_area_right_start[0], goal_area_right_end[1]),
                                    (goal_area_right_start[0] - field_feature_size, goal_area_right_end[1]),
                                    color, stroke_width)
            
            # draw center line to side line t crossings
            # top
            img = cv2.line(img, (middle_line_start[0] - field_feature_size // 2, middle_line_start[1]),
                                    (middle_line_start[0] + field_feature_size // 2, middle_line_start[1]),
                                    color, stroke_width)
            img = cv2.line(img, middle_line_start,
                                    (middle_line_start[0], middle_line_start[1] + field_feature_size),
                                    color, stroke_width)

            # bottom
            img = cv2.line(img, (middle_line_end[0] - field_feature_size // 2, middle_line_end[1]),
                                    (middle_line_end[0] + field_feature_size // 2, middle_line_end[1]),
                                    color, stroke_width)
            img = cv2.line(img, middle_line_end,
                                    (middle_line_end[0], middle_line_end[1] - field_feature_size),
                                    color, stroke_width)

                                            

        if target in [MapTypes.TCROSSINGS, MapTypes.FIELD_FEATURES] and field_feature_style == FieldFeatureStyles.BLOB:
            # draw blobs for goal areas
            img = cv2.circle(img, goal_area_left_start, field_feature_size, color, -1)
            img = cv2.circle(img, (goal_area_left_start[0], goal_area_left_end[1]), field_feature_size, color, -1)
            img = cv2.circle(img, goal_area_right_start, field_feature_size, color, -1)
            img = cv2.circle(img, (goal_area_right_start[0], goal_area_right_end[1]), field_feature_size, color, -1)

            # middle line
            img = cv2.circle(img, middle_line_start, field_feature_size, color, -1)
            img = cv2.circle(img, middle_line_end, field_feature_size, color, -1)

        if target in [MapTypes.CROSSES, MapTypes.FIELD_FEATURES] and field_feature_style == FieldFeatureStyles.EXACT:
            # penalty marks
            if penalty_mark: 
                self.drawCross(img, penalty_mark_left, color, stroke_width, field_feature_size // 2)
                self.drawCross(img, penalty_mark_right, color, stroke_width, field_feature_size // 2)

            # middle point
            if center_point:
                self.drawCross(img, middle_point, color, stroke_width, field_feature_size // 2)

            # center circle middle line crossings
            self.drawCross(img, (middle_point[0], middle_point[1] - center_circle_diameter), color, stroke_width, field_feature_size // 2)
            self.drawCross(img, (middle_point[0], middle_point[1] + center_circle_diameter), color, stroke_width, field_feature_size // 2)

        if target in [MapTypes.CROSSES, MapTypes.FIELD_FEATURES] and field_feature_style == FieldFeatureStyles.BLOB:
            # penalty marks
            if penalty_mark:
                img = cv2.circle(img, penalty_mark_left, field_feature_size, color, -1)
                img = cv2.circle(img, penalty_mark_right, field_feature_size, color, -1)

            # middle point
            if center_point:
                img = cv2.circle(img, middle_point, field_feature_size, color, -1)

            # center circle middle line crossings
            img = cv2.circle(img, (middle_point[0], middle_point[1] - center_circle_diameter), field_feature_size, color,
                                    -1)
            img = cv2.circle(img, (middle_point[0], middle_point[1] + center_circle_diameter), field_feature_size, color,
                                    -1)

        # Blur
        img = self.blurDistance(img, blur_factor) # TODO

        # Invert
        if invert:
            img = 255 - img

        return img

    def display_map(self, image):
        # Display the generated map on the canvas
        img = Image.fromarray(image)
        # Resize to fit canvas while keeping aspect ratio
        img.thumbnail((self.canvas.winfo_width(), self.canvas.winfo_height()), Image.Resampling.LANCZOS)
        img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=img)
        self.canvas.image = img  # To prevent garbage collection


if __name__ == "__main__":
    root = tk.Tk()
    app = MapGeneratorGUI(root)
    root.mainloop()
