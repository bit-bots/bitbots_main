import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from PIL import Image, ImageTk
import cv2
import math
import numpy as np
from scipy import ndimage
from enum import Enum
import yaml
import os

from tooltip import Tooltip


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

            # Add tooltips
            Tooltip(label, text=parameter_definition["tooltip"])
            Tooltip(ui_element, text=parameter_definition["tooltip"])

            # Add ui elements to the dict
            self.parameter_ui_elements[parameter_name] = {
                "label": label,
                "ui_element": ui_element
            }

            # Store variable for later state access
            self.parameter_values[parameter_name] = variable


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

        # Title
        self.title = ttk.Label(self.root, text="Soccer Map Generator", font=("TkDefaultFont", 16))

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
                "tooltip": "Whether or not to draw the penalty mark"
            },
            "center_point": {
                "type": bool,
                "default": True,
                "label": "Center Point",
                "tooltip": "Whether or not to draw the center point"
            },
            "goal_back": {
                "type": bool,
                "default": True,
                "label": "Goal Back",
                "tooltip": "Whether or not to draw the back area of the goal"
            },
            "stroke_width": {
                "type": int,
                "default": 5,
                "label": "Stoke Width",
                "tooltip": "Width (in px) of the shapes we draw"
            },
            "field_length": {
                "type": int,
                "default": 900,
                "label": "Field Length",
                "tooltip": "Length of the field in cm"
            },
            "field_width": {
                "type": int,
                "default": 600,
                "label": "Field Width",
                "tooltip": "Width of the field in cm"
            },
            "goal_depth": {
                "type": int,
                "default": 60,
                "label": "Goal Depth",
                "tooltip": "Depth of the goal in cm"
            },
            "goal_width": {
                "type": int,
                "default": 260,
                "label": "Goal Width",
                "tooltip": "Width of the goal in cm"
            },
            "goal_area_length": {
                "type": int,
                "default": 100,
                "label": "Goal Area Length",
                "tooltip": "Length of the goal area in cm"
            },
            "goal_area_width": {
                "type": int,
                "default": 300,
                "label": "Goal Area Width",
                "tooltip": "Width of the goal area in cm"
            },
            "penalty_mark_distance": {
                "type": int,
                "default": 150,
                "label": "Penalty Mark Distance",
                "tooltip": "Distance of the penalty mark from the goal line in cm"
            },
            "center_circle_diameter": {
                "type": int,
                "default": 150,
                "label": "Center Circle Diameter",
                "tooltip": "Diameter of the center circle in cm"
            },
            "border_strip_width": {
                "type": int,
                "default": 100,
                "label": "Border Strip Width",
                "tooltip": "Width of the border strip around the field in cm"
            },
            "penalty_area_length": {
                "type": int,
                "default": 200,
                "label": "Penalty Area Length",
                "tooltip": "Length of the penalty area in cm"
            },
            "penalty_area_width": {
                "type": int,
                "default": 500,
                "label": "Penalty Area Width",
                "tooltip": "Width of the penalty area in cm"
            },
            "field_feature_size": {
                "type": int,
                "default": 30,
                "label": "Field Feature Size",
                "tooltip": "Size of the field features in cm"
            },
            "mark_type": {
                "type": MarkTypes,
                "default": MarkTypes.CROSS,
                "label": "Mark Type",
                "tooltip": "Type of the marks (penalty mark, center point)"
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
                "tooltip": "Magic value to blur the distance map"
            },
            "invert": {
                "type": bool,
                "default": True,
                "label": "Invert",
                "tooltip": "Invert the final image"
            }
        })

        # Generate Map Button
        self.save_map_button = ttk.Button(self.root, text="Save Map", command=self.save_map)

        # Save metadata checkbox
        self.save_metadata = tk.BooleanVar(value=True)
        self.save_metadata_checkbox = ttk.Checkbutton(self.root, text="Save Metadata", variable=self.save_metadata)

        # Load and save config buttons
        self.load_config_button = ttk.Button(self.root, text="Load Config", command=self.load_config)
        self.save_config_button = ttk.Button(self.root, text="Save Config", command=self.save_config)

        # Canvas to display the generated map
        self.canvas = tk.Canvas(self.root, width=800, height=600)

        # Layout

        # Parameter input and generate button
        self.title.grid(row=0, column=0, columnspan=2, pady=20, padx=10)
        self.parameter_input.grid(row=1, column=0, columnspan=2, pady=10, padx=10)
        self.load_config_button.grid(row=2, column=0, columnspan=1, pady=10)
        self.save_config_button.grid(row=2, column=1, columnspan=1, pady=10)
        self.save_metadata_checkbox.grid(row=3, column=0, columnspan=1, pady=10)
        self.save_map_button.grid(row=3, column=1, columnspan=1, pady=10)

        # Preview
        self.canvas.grid(row=0, column=2, rowspan=4, pady=10, padx=30)

        # Color in which we want to draw the lines
        self.primary_color = (255, 255, 255)  # white

        # Render initial map
        self.root.update()  # We need to call update() first
        self.update_map()

    def load_config(self):
        # Prompt the user to select a file (force yaml)
        file = filedialog.askopenfile(
            mode='r',
            defaultextension=".yaml",
            filetypes=(("yaml file", "*.yaml"), ("All Files", "*.*")))
        if file:
            # Load the parameters from the file
            config_file = yaml.load(file, Loader=yaml.FullLoader)
            # Check if the file is valid (has the correct fields)
            if "header" not in config_file \
                    or "type" not in config_file["header"] \
                    or "version" not in config_file["header"] \
                    or "parameters" not in config_file \
                    or config_file["header"]["version"] != "1.0" \
                    or config_file["header"]["type"] != "map_generator_config":
                # Show error box and return if the file is invalid
                tk.messagebox.showerror("Error", "Invalid config file")
                return
            # Set the parameters in the gui
            for parameter_name, parameter_value in config_file["parameters"].items():
                self.parameter_input.parameter_values[parameter_name].set(parameter_value)
            # Update the map
            self.update_map()

    def save_config(self):
        # Get the parameters from the user input
        parameters = self.parameter_input.get_parameters()
        # Open a file dialog to select the file
        file = filedialog.asksaveasfile(
            mode='w',
            defaultextension=".yaml",
            filetypes=(("yaml file", "*.yaml"), ("All Files", "*.*")))
        if file:
            # Add header
            file.write("# Map Generator Config\n")
            file.write("# This file was generated by the map generator GUI\n")
            file.write("# You can load it in the GUI to restore the parameters\n\n")
            # Save the parameters in this format:
            yaml.dump({
                "header": {
                    "version": "1.0",
                    "type": "map_generator_config"
                },
                "parameters": parameters
            }, file, sort_keys=False)
            print(f"Saved config to {file.name}")

    def save_map(self):
        file = filedialog.asksaveasfile(
            mode='w',
            defaultextension=".png",
            filetypes=(("png file", "*.png"), ("All Files", "*.*")))
        if file:
            print(f"Saving map to {file.name}")

            # Generate and save the map
            generated_map = self.generate_map_image()
            if cv2.imwrite(file.name, generated_map):
                # Save metadata
                if self.save_metadata.get():
                    # Save the metadata in this format:
                    metadata = self.generate_metadata(os.path.basename(file.name))
                    # Save metadata in the same folder as the map
                    metadata_file = os.path.join(os.path.dirname(file.name), "metadata.yaml")
                    with open(metadata_file, "w") as f:
                        yaml.dump(metadata, f, sort_keys=False)
                    print(f"Saved metadata to {metadata_file}")


                # Show success box and ask if we want to open it with the default image viewer
                if tk.messagebox.askyesno("Success", "Map saved successfully. Do you want to open it?"):
                    import platform
                    import subprocess
                    if platform.system() == "Windows":
                        subprocess.Popen(["start", file.name], shell=True)
                    elif platform.system() == "Darwin":
                        subprocess.Popen(["open", file.name])
                    else:
                        subprocess.Popen(["xdg-open", file.name])
            else:
                # Show error box
                tk.messagebox.showerror("Error", "Could not save map to file")

    def generate_metadata(self, image_name: str) -> dict:
        # Get the parameters from the user input
        parameters = self.parameter_input.get_parameters()

        # Get the field dimensions in cm
        field_dimensions = np.array([parameters["field_length"], parameters["field_width"], 0])
        # Add the border strip
        field_dimensions[:2] += 2 * parameters["border_strip_width"]
        # Get the origin
        origin = -field_dimensions / 2
        # Convert to meters
        origin /= 100

        # Generate the metadata
        metadata = {
            "image": image_name,
            "resolution": 0.01,
            "origin": origin.tolist(),
            "occupied_thresh": 0.99,
            "free_thresh": 0.196,
            "negate": int(parameters["invert"])
        }
        return metadata

    def update_map(self, *args):
        # Generate and display the map on the canvas
        try:
            generated_map = self.generate_map_image()
            self.display_map(generated_map)
        except tk.TclError as e:
            print(f"Invalid input for map generation. '{e}'")

    def drawCross(self, img, point, color, width=5, length=15):
        half_width = width // 2
        vertical_start = (point[0], point[1] - length)
        vertical_end = (point[0], point[1] + length)
        horizontal_start = (point[0] - length, point[1])
        horizontal_end = (point[0] + length, point[1])
        img = cv2.line(img, vertical_start, vertical_end, color, width)
        img = cv2.line(img, horizontal_start, horizontal_end, color, width)

    def blurDistance(self, image, blur_factor):
        # Calc distances
        distance_map = 255 - ndimage.distance_transform_edt(255 - image)

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
        parameters = self.parameter_input.get_parameters()
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
        blur_factor = parameters["blur_factor"]
        invert = parameters["invert"]
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
        if blur_factor > 0:
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
