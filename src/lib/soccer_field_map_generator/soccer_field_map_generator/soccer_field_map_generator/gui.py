# Copyright (c) 2023 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum
import os
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk

import cv2
from PIL import Image, ImageTk
from soccer_field_map_generator.generator import (
    FieldFeatureStyles,
    generate_map_image,
    generate_metadata,
    load_config_file,
    MapTypes,
    MarkTypes,
)
from soccer_field_map_generator.tooltip import Tooltip
import yaml


class MapGeneratorParamInput(tk.Frame):

    def __init__(
        self, parent, update_hook: callable, parameter_definitions: dict[str, dict]
    ):
        tk.Frame.__init__(self, parent)

        # Keep track of parameter definitions, GUI elements and the input values
        self.parameter_definitions = parameter_definitions
        self.parameter_ui_elements: dict[str, dict[str, ttk.Widget]] = {}
        self.parameter_values: dict[str, tk.Variable] = {}

        # Generate GUI elements for all parameters
        for parameter_name, parameter_definition in parameter_definitions.items():
            # Create GUI elements
            label = ttk.Label(self, text=parameter_definition['label'])
            if parameter_definition['type'] == bool:
                variable = tk.BooleanVar(value=parameter_definition['default'])
                ui_element = ttk.Checkbutton(
                    self, command=update_hook, variable=variable
                )
            elif parameter_definition['type'] == int:
                variable = tk.IntVar(value=parameter_definition['default'])
                ui_element = ttk.Entry(self, textvariable=variable)
                ui_element.bind('<KeyRelease>', update_hook)
            elif parameter_definition['type'] == float:
                variable = tk.DoubleVar(value=parameter_definition['default'])
                ui_element = ttk.Entry(self, textvariable=variable)
                ui_element.bind('<KeyRelease>', update_hook)
            elif issubclass(parameter_definition['type'], Enum):
                variable = tk.StringVar(value=parameter_definition['default'].value)
                values = [enum.value for enum in parameter_definition['type']]
                ui_element = ttk.Combobox(self, values=values, textvariable=variable)
                ui_element.bind('<<ComboboxSelected>>', update_hook)
            else:
                raise NotImplementedError('Parameter type not implemented')

            # Add tooltips
            Tooltip(label, text=parameter_definition['tooltip'])
            Tooltip(ui_element, text=parameter_definition['tooltip'])

            # Add ui elements to the dict
            self.parameter_ui_elements[parameter_name] = {
                'label': label,
                'ui_element': ui_element,
            }

            # Store variable for later state access
            self.parameter_values[parameter_name] = variable

        # Create layout
        for i, parameter_name in enumerate(parameter_definitions.keys()):
            self.parameter_ui_elements[parameter_name]['label'].grid(
                row=i, column=0, sticky='e'
            )
            self.parameter_ui_elements[parameter_name]['ui_element'].grid(
                row=i, column=1, sticky='w'
            )

    def get_parameters(self):
        return {
            parameter_name: parameter_value.get()
            for parameter_name, parameter_value in self.parameter_values.items()
        }

    def get_parameter(self, parameter_name):
        return self.parameter_values[parameter_name].get()


class MapGeneratorGUI:

    def __init__(self, root: tk.Tk):
        # Set ttk theme
        s = ttk.Style()
        s.theme_use('clam')

        # Set window title and size
        self.root = root
        self.root.title('Map Generator GUI')
        self.root.resizable(False, False)

        # Create GUI elements

        # Title
        self.title = ttk.Label(
            self.root, text='Soccer Map Generator', font=('TkDefaultFont', 16)
        )

        # Parameter Input
        self.parameter_input = MapGeneratorParamInput(
            self.root,
            self.update_map,
            {
                'map_type': {
                    'type': MapTypes,
                    'default': MapTypes.LINE,
                    'label': 'Map Type',
                    'tooltip': 'Type of the map we want to generate',
                },
                'penalty_mark': {
                    'type': bool,
                    'default': True,
                    'label': 'Penalty Mark',
                    'tooltip': 'Whether or not to draw the penalty mark',
                },
                'center_point': {
                    'type': bool,
                    'default': True,
                    'label': 'Center Point',
                    'tooltip': 'Whether or not to draw the center point',
                },
                'goal_back': {
                    'type': bool,
                    'default': True,
                    'label': 'Goal Back',
                    'tooltip': 'Whether or not to draw the back area of the goal',
                },
                'stroke_width': {
                    'type': int,
                    'default': 5,
                    'label': 'Stoke Width',
                    'tooltip': 'Width (in px) of the shapes we draw',
                },
                'field_length': {
                    'type': int,
                    'default': 900,
                    'label': 'Field Length',
                    'tooltip': 'Length of the field in cm',
                },
                'field_width': {
                    'type': int,
                    'default': 600,
                    'label': 'Field Width',
                    'tooltip': 'Width of the field in cm',
                },
                'goal_depth': {
                    'type': int,
                    'default': 60,
                    'label': 'Goal Depth',
                    'tooltip': 'Depth of the goal in cm',
                },
                'goal_width': {
                    'type': int,
                    'default': 260,
                    'label': 'Goal Width',
                    'tooltip': 'Width of the goal in cm',
                },
                'goal_area_length': {
                    'type': int,
                    'default': 100,
                    'label': 'Goal Area Length',
                    'tooltip': 'Length of the goal area in cm',
                },
                'goal_area_width': {
                    'type': int,
                    'default': 300,
                    'label': 'Goal Area Width',
                    'tooltip': 'Width of the goal area in cm',
                },
                'penalty_mark_distance': {
                    'type': int,
                    'default': 150,
                    'label': 'Penalty Mark Distance',
                    'tooltip': 'Distance of the penalty mark from the goal line in cm',
                },
                'center_circle_diameter': {
                    'type': int,
                    'default': 150,
                    'label': 'Center Circle Diameter',
                    'tooltip': 'Diameter of the center circle in cm',
                },
                'border_strip_width': {
                    'type': int,
                    'default': 100,
                    'label': 'Border Strip Width',
                    'tooltip': 'Width of the border strip around the field in cm',
                },
                'penalty_area_length': {
                    'type': int,
                    'default': 200,
                    'label': 'Penalty Area Length',
                    'tooltip': 'Length of the penalty area in cm',
                },
                'penalty_area_width': {
                    'type': int,
                    'default': 500,
                    'label': 'Penalty Area Width',
                    'tooltip': 'Width of the penalty area in cm',
                },
                'field_feature_size': {
                    'type': int,
                    'default': 30,
                    'label': 'Field Feature Size',
                    'tooltip': 'Size of the field features in cm',
                },
                'mark_type': {
                    'type': MarkTypes,
                    'default': MarkTypes.CROSS,
                    'label': 'Mark Type',
                    'tooltip': 'Type of the marks (penalty mark, center point)',
                },
                'field_feature_style': {
                    'type': FieldFeatureStyles,
                    'default': FieldFeatureStyles.EXACT,
                    'label': 'Field Feature Style',
                    'tooltip': 'Style of the field features',
                },
                'distance_map': {
                    'type': bool,
                    'default': False,
                    'label': 'Distance Map',
                    'tooltip': 'Whether or not to draw the distance map',
                },
                'distance_decay': {
                    'type': float,
                    'default': 0.0,
                    'label': 'Distance Decay',
                    'tooltip': 'Exponential decay applied to the distance map',
                },
                'invert': {
                    'type': bool,
                    'default': True,
                    'label': 'Invert',
                    'tooltip': 'Invert the final image',
                },
            },
        )

        # Generate Map Button
        self.save_map_button = ttk.Button(
            self.root, text='Save Map', command=self.save_map
        )

        # Save metadata checkbox
        self.save_metadata = tk.BooleanVar(value=True)
        self.save_metadata_checkbox = ttk.Checkbutton(
            self.root, text='Save Metadata', variable=self.save_metadata
        )

        # Load and save config buttons
        self.load_config_button = ttk.Button(
            self.root, text='Load Config', command=self.load_config
        )
        self.save_config_button = ttk.Button(
            self.root, text='Save Config', command=self.save_config
        )

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
            defaultextension='.yaml',
            filetypes=(('yaml file', '*.yaml'), ('All Files', '*.*')),
        )
        if file:
            # Load the config file
            config = load_config_file(file)
            if config is None:
                # Show error box and return if the file is invalid
                tk.messagebox.showerror('Error', 'Invalid config file')
                return
            # Set the parameters in the gui
            for parameter_name, parameter_value in config.items():
                self.parameter_input.parameter_values[parameter_name].set(
                    parameter_value
                )
            # Update the map
            self.update_map()

    def save_config(self):
        # Get the parameters from the user input
        parameters = self.parameter_input.get_parameters()
        # Open a file dialog to select the file
        file = filedialog.asksaveasfile(
            mode='w',
            defaultextension='.yaml',
            filetypes=(('yaml file', '*.yaml'), ('All Files', '*.*')),
        )
        if file:
            # Add header
            file.write('# Map Generator Config\n')
            file.write('# This file was generated by the map generator GUI\n\n')
            # Save the parameters in this format:
            yaml.dump(
                {
                    'header': {'version': '1.0', 'type': 'map_generator_config'},
                    'parameters': parameters,
                },
                file,
                sort_keys=False,
            )
            print(f'Saved config to {file.name}')

    def save_map(self):
        file = filedialog.asksaveasfile(
            mode='w',
            defaultextension='.png',
            filetypes=(('png file', '*.png'), ('All Files', '*.*')),
        )
        if file:
            print(f'Saving map to {file.name}')

            # Generate and save the map
            parameters = self.parameter_input.get_parameters()
            generated_map = generate_map_image(parameters)
            if cv2.imwrite(file.name, generated_map):
                # Save metadata
                if self.save_metadata.get():
                    # Save the metadata in this format:
                    metadata = generate_metadata(
                        parameters, os.path.basename(file.name)
                    )
                    # Save metadata in the same folder as the map
                    metadata_file = os.path.join(
                        os.path.dirname(file.name), 'map_server.yaml'
                    )
                    with open(metadata_file, 'w') as f:
                        yaml.dump(metadata, f, sort_keys=False)
                    print(f'Saved metadata to {metadata_file}')

                # Show success box and ask if we want to open it with the default image viewer
                if tk.messagebox.askyesno(
                    'Success', 'Map saved successfully. Do you want to open it?'
                ):
                    import platform
                    import subprocess

                    if platform.system() == 'Windows':
                        subprocess.Popen(['start', file.name], shell=True)
                    elif platform.system() == 'Darwin':
                        subprocess.Popen(['open', file.name])
                    else:
                        subprocess.Popen(['xdg-open', file.name])
            else:
                # Show error box
                tk.messagebox.showerror('Error', 'Could not save map to file')

    def update_map(self, *args):
        # Generate and display the map on the canvas
        try:
            generated_map = generate_map_image(self.parameter_input.get_parameters())
            self.display_map(generated_map)
        except tk.TclError as e:
            print(f"Invalid input for map generation. '{e}'")

    def display_map(self, image):
        # Display the generated map on the canvas
        img = Image.fromarray(image)
        # Resize to fit canvas while keeping aspect ratio
        img.thumbnail(
            (self.canvas.winfo_width(), self.canvas.winfo_height()),
            Image.Resampling.LANCZOS,
        )
        img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=img)
        self.canvas.image = img  # To prevent garbage collection


def main():
    root = tk.Tk()
    MapGeneratorGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
