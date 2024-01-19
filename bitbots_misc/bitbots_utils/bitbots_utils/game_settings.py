#!/usr/bin/env python3

import os
from pydoc import locate
from typing import Any, Optional

import yaml

YAML_COMPATIBLE_SCALAR_TYPES = [int, float, str, bool]

# path to the game settings yaml and to the game setting options
SETTING_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "bitbots_utils",
    "config",
    "game_settings.yaml",
)
OPTIONS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "bitbots_utils",
    "config",
    "game_settings_options.yaml",
)


def provide_config(path):
    """
    reads out the yaml you are asking for with the path parameter
    :param path: filepath for your yaml
    :return: config as dict
    """
    if os.path.exists(path):
        try:
            with open(path) as f:
                config = yaml.load(f, Loader=yaml.UnsafeLoader)
        except yaml.YAMLError as exc:
            print("Error in configuration file:", exc)
    else:
        config = {}
        print(f"The config yaml with path {path}, does not exist.")

    return config


def ask_for_config_option(
    name: str,
    value_type: Any,
    current_value: Any = None,
    valid_options: Optional[list[Any]] = None,
    explanation: Optional[str] = None,
) -> object:
    """
    :param name: name of the config-option-value e.g. robot number
    :param value_type: type object of the type of value to cast to
    :param valid_options: possible options for the value, type of input
    :param current_value: the already set value
    :param explanation: describes options
    :return: new chosen value for this config option, can be the old one
    """
    print(f"=============== {name} ===============")
    if valid_options:
        print(f"Options: {valid_options}")
    print(f"Explanations: {explanation}")

    if current_value is not None:
        input_prompt = f"Value ({current_value}): "
    else:
        input_prompt = "Value: "

    value_is_valid = False
    while not value_is_valid:
        new_value = input(input_prompt).lower()

        if new_value == "":
            if current_value is not None:
                new_value = current_value
                value_is_valid = True
        else:
            value_is_valid = check_new_value(new_value, value_type, valid_options)

    if value_type in YAML_COMPATIBLE_SCALAR_TYPES:
        return value_type(new_value)
    else:
        return str(new_value)


def check_new_value(new_value: str, value_type: Any, valid_options: Optional[list[Any]] = None) -> bool:
    """
    checks with definition if new value is a valid input
    :param new_value: input to set as new value
    :param value_type: type object of the type of value to cast to
    :param valid_options: valid options for new value
    :return: true if valid, false if not
    """

    try:
        new_value = value_type(new_value)
    except Exception:
        print(f"{new_value} could not be converted to a {value_type}. Are you sure it is in the right format?")
        return False

    not_a_valid_option = valid_options is not None and new_value not in valid_options
    if not_a_valid_option:
        print(f" {new_value} no valid option")
        return False

    return True


def main():
    config = provide_config(SETTING_PATH)
    ros_parameters = config["parameter_blackboard"]["ros__parameters"]
    if ros_parameters is None:
        ros_parameters = {}
        config["parameter_blackboard"]["ros__parameters"] = ros_parameters

    options = provide_config(OPTIONS_PATH)
    for key in options.keys():
        entry_value_type = locate(options[key]["type"])
        entry_options = options[key].get("options", None)
        entry_explanation = options[key]["explanation"]

        if key in ros_parameters.keys():
            ros_parameters[key] = ask_for_config_option(
                key, entry_value_type, ros_parameters[key], entry_options, entry_explanation
            )
        else:
            value = ask_for_config_option(key, entry_value_type, None, entry_options, entry_explanation)
            ros_parameters.update({key: value})

    with open(SETTING_PATH, "w") as f:
        yaml.safe_dump(config, f, default_flow_style=False)


if __name__ == "__main__":
    main()
