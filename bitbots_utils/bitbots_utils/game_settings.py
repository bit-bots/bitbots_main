#!/usr/bin/env python3
import sys

import yaml
import os


# path to the game settings yaml and to the game setting options
SETTING_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                            "bitbots_utils", "config", "game_settings.yaml")
OPTIONS_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                              "bitbots_utils","config", "game_settings_options.yaml")
def provide_config(path):
    """
    reads out the yaml you are asking for with the path parameter
    :param path: filepath for your yaml
    :return: config as dict
    """
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                config = yaml.load(f, Loader=yaml.UnsafeLoader)
        except yaml.YAMLError as exc:
            print("Error in configuration file:", exc)
    else:
        config = {}
        print("The config yaml with path {}, does not exist.".format(path))

    return config


def ask_for_config_option(name: object, definition: object, current_value: object = None, explanation: object = None) -> object:
    """
    :param name: name of the config-option-value e.g. robot number
    :param definition: possible options for the value, type of input
    :param current_value: the already set value
    :param explanation: describes options
    :return: new chosen value for this config option, can be the old one
    """
    print('=============== {} ==============='.format(name))
    if type(definition) is range:
        definition = list(definition)
    print("Options: {}".format(definition))
    print("Explanations: {}".format(explanation))
    if current_value is not None:
        input_prompt = 'Value ({}): '.format(current_value)
    else:
        input_prompt = 'Value: '

    value_is_valid = False
    while not value_is_valid:
        new_value = input(input_prompt).lower()

        if new_value == '':
            if current_value is not None:
                new_value = current_value
                value_is_valid = True
        else:
            value_is_valid = check_new_value(new_value, definition)
    def_type = type(definition[0])
    return def_type(new_value)


def check_new_value(new_value: str, definition) -> bool:
    """
    checks with definition if new value is a valid input
    :param new_value: input to set as new value
    :param definition: valid options for new value
    :return: true if valid, false if not
    """

    

    definitiontype = type(definition[0])

    try:
        new_value = definitiontype(new_value) # casts value to the type of
    except:
        print("{} could not be converted to a {}. Are you sure it is in the right format?".format(new_value,definitiontype))


    if new_value in definition:
        return True
    else:
        # print(new_value, definition)
        print(' {} no valid option'.format(new_value))
        return False



def main():
    config = provide_config(SETTING_PATH)
    #config = config['parameter_blackboard']['ros_parameters']
    ros_parameters = config['parameter_blackboard']['ros__parameters']
    if ros_parameters is None:
        ros_parameters = {}
        config['parameter_blackboard']['ros__parameters'] = ros_parameters
        
    options = provide_config(OPTIONS_PATH)
    
    for key in options.keys():
        if key in ros_parameters.keys():
            ros_parameters[key] = ask_for_config_option(key, options[key]['options'], ros_parameters[key],
                                                options[key]['explanation'])
        else:
            value = ask_for_config_option(key, options[key]['options'], None,
                                                options[key]['explanation'])
            ros_parameters.update({key : value})


    with open(SETTING_PATH, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)


if __name__ == '__main__':
    main()
