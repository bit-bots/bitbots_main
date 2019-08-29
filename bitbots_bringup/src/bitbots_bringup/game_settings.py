#!/usr/bin/python3
import sys

import yaml
import rospy
import rospkg
import os
import roslaunch

rospack = rospkg.RosPack()

# every parameter has its own SETTING_PATH
SETTING_PATH = rospack.get_path("bitbots_bringup") + "/config/game_settings.yaml"
TO_BE_SET_PATH = rospack.get_path("bitbots_bringup") + "/config/to_be_set_game_settings.yaml"


def provide_config(path):
    """
    reads out the yaml you are asking for with the path parameter
    :param path: filepath for your yaml
    :return: config as dict
    """
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                config = yaml.load(f)
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
            new_value = current_value
            value_is_valid = True
        else:
            value_is_valid = check_new_value(new_value, definition)

    print()
    def_type = type(definition[0])
    return def_type(new_value)


def check_new_value(new_value: str, definition) -> bool:
    """
    checks with definition if new value is a valid input
    :param new_value: input to set as new value
    :param definition: valid options for new value
    :return: true if valid, false if not
    """

    if type(definition) is range:
        definition = list(definition)

    if definition == "custom":
        return True

    definitiontype = type(definition[0])

    try:
        new_value = definitiontype(new_value) # casts value to the type of
    except:
        print("{} could not be converted to a {}. Are you sure it is in the right format?".format(new_value,definitiontype))

    if type(definition) is list:
        if new_value in definition:
            return True
        else:
            # print(new_value, definition)
            print(' {} no valid option'.format(new_value))
            # print(type(definition[0]))
            return False

    elif definition is bool:
        if new_value == "true" or new_value == "false":
            return True
        else:
            return False

    elif definition is int:
        try:
            int(new_value)
            return True
        except ValueError:
            return False

    elif definition is float:
        try:
            float(new_value)
            return True
        except ValueError:
            return False

    elif definition is str:
        return True

    else:
        # We could not validate the type or values so we assume it is incorrect
        return False


def main():
    rospy.init_node("game_settings")
    config = provide_config(SETTING_PATH)

    # every option for a config-value is listed here
    '''
    options = {
        #'playernumber': {'package': 'bla', 'file': 'doc', 'parameter': 'playernumber', 'options': ['1', '2', '3', '4']},
        'bot_id': {'package': 'humanoid_league_game_controller', 'file': '/config/game_controller.yaml', 'options': ['1', '2', '3', '4', '5']},
        'team_id': {'package': 'humanoid_league_game_controller', 'file': '/config/game_controller.yaml', 'options': ['1', '2', '3', '4', '5']}
    }
    '''
    to_be_set = provide_config(TO_BE_SET_PATH)

    for key, value in to_be_set.items():
        if key in config.keys():
            config[key] = ask_for_config_option(key, to_be_set[key]['options'], config[key],
                                                to_be_set[key]['explanation'])
        else:
            config[key] = to_be_set[key].copy()
            del config[key]['options']
            del config[key]['explanation']
            del config[key]['file']
            del config[key]['package']
            config[key] = ask_for_config_option(key, to_be_set[key]['options'], to_be_set[key]['explanation'])

    with open(SETTING_PATH, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    if len(sys.argv) == 1 or sys.argv[1] != '--no-teamplayer':
        start_teamplayer = input("Do you want to launch 'teamplayer.launch'? (y/N)")
        if start_teamplayer.lower() == "y":
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [
                rospack.get_path("bitbots_bringup") + "/launch/teamplayer.launch"])
            launch.start()


if __name__ == '__main__':
    main()
