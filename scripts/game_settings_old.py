import yaml
# import rospy
# import rospack
import os


# every parameter has its own SETTING_PATH
SETTING_PATH = os.path.expanduser("gamesettings_test.yaml")

def provide_config():
    if os.path.exists(SETTING_PATH):
        try:
            with open(SETTING_PATH, 'r') as f:
                config = yaml.load(f)
        except yaml.YAMLError as exc:
            print("Error in configuration file:", exc)
    else:
        config = {}

    return config


def ask_for_config_option(name: str, definition, current_value: str = None):
    """
    :param name: name of the config-option-value e.g. robot number
    :param definition: possible options for the value, type of input
    :param current_value: the already set value
    :return: new chosen value for this config option, can be the old one
    """
    print('=============== {} ==============='.format(name))

    print("Options: {}".format(definition))
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
    return parse_to_type(new_value, definition)


def parse_to_type(value, definition):
    """
    parses any value according to the specified definition
    :param value: the value you want to parse
    :param definition: the specified type you want the value parsed to
    :return: the type of the value
    """

    if definition is bool:
        return bool(value)

    elif definition is int:
        return int(value)

    elif definition is float:
        return float(value)

    elif definition is str:
        return value

    else:
        return value


def check_new_value(new_value: str, definition) -> bool:
    """
    checks with definition if new value is a valid input
    :param new_value: input to set as new value
    :param definition: valid options for new value
    :return: true if valid, false if not
    """

    if type(definition) is list:
        if new_value in definition:
            return True
        else:
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


if __name__ == '__main__':
    config = provide_config()

    # every option for a config-value is listed here
    options = {
        'a': bool,
        'number': int,
        'colour': ['b', 'r'],
        'name': str
    }

    for key, value in options.items():
        if key in config.keys():
            config[key] = ask_for_config_option(key, value, config[key])
        else:
            config[key] = ask_for_config_option(key, value)

    with open(SETTING_PATH, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
