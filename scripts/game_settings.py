import yaml
import rospy
import rospack
import os


CONFIG_DETAILS_PATH = rospack.get_path("game_settings_config_details.yaml")
GAME_SETTINGS_PATH = rospack.get_path("game_settings.yaml")


def provide_config_details():
    """
    converts the yaml to a dictionary, hierarchy is represented through nested dictionaries
    :return: yaml file as python dictionary
    """
    global config_details
    try:
        with open(CONFIG_DETAILS_PATH, 'r') as f:
            config_details = yaml.load(f)
    except yaml.YAMLError as exc:
        print("Error in configuration file:", exc)

    return config_details


def provide_last_game_config():
    """
    if you already executed the script, an old game_settings.yaml exists,
    which you can get the current values from
    :return: if existing last config, else empty dictionary
    """
    config = {}
    if os.path.exists(GAME_SETTINGS_PATH):
        try:
            with open(GAME_SETTINGS_PATH, 'r') as g:
                config = yaml.load(g)
        except yaml.YAMLError as exc:
            print("Error in configuration file:", exc)

    return config


def get_standard_value(package, file, parameter: str):
    """
    DOES NOT WORK YET
    looks for the currently set value, if there is one
    :param package: the package you can find the paramter in
    :param file: the file you can find the parameter in
    :param parameter: the name of the parameter
    :return: the current value, if it is set
    """

    if new_config is not None:
        standard_value = new_config[parameter]
        return standard_value
    # don't know if the following actually works ...
    else:
        parameter_path = os.path.expanduser(
            '{}/{}'.format(package, file))  # rospack.get_path('{}/{}'.format(package, file))
        parameter_config = yaml.load(parameter_path)
        parts = parameter.split("/")
        # the nested structure from the yaml file converted into python dictionary structure
        p_in_yaml: str = "parameter_config[{}]".format(parts[0])

        for i in range(1, len(parts)):
            p_in_yaml += "[{}]".format(parts[i])

        standard_value = exec(p_in_yaml)
        return standard_value


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


def ask_for_config(parameter: str, options: list, current_value: str = None):
    """
    # :param package: the package the parameter is saved in 
    # :param file: name of the file which the parameter is in
    :param parameter: name of the parameter
    :param options: all valid options for the parameter 
    :return: new value
    """""
    print('=============== {} ==============='.format(parameter))

    print("Options: {}".format(options))
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
            value_is_valid = check_new_value(new_value, options)

    print()
    return new_value


if __name__ == '__main__':
    # reads out the file containing important information about parameters
    config_details = provide_config_details()

    # last setted parameters
    new_config = provide_last_game_config()

    for key in config_details.keys():
        package = config_details[key]['package']
        file = config_details[key]['file']
        parameter = config_details[key]['parameter']
        options = config_details[key]['options']
        # Ic currently not working
        # standard_value = get_standard_value(package, file, parameter)
        if parameter in new_config.keys():
            current_value = new_config[parameter]
            new_config[parameter] = ask_for_config(parameter, options, current_value)
        else:
            new_config[parameter] = ask_for_config(parameter, options)

    # the file all parameters should be set in
    # will be converted in yaml
    with open(GAME_SETTINGS_PATH, 'w') as f:
        yaml.dump(new_config, f, default_flow_style=False)
