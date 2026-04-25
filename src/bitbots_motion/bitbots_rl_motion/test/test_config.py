from pathlib import Path

import yaml  # type: ignore[import-untyped]


def test_walking_config():
    config_path = Path(__file__).parent.parent / "configs" / "wolfgang_walking_model_config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)

    params = config["walk_node"]["ros__parameters"]

    assert isinstance(params["model"], str)
    assert isinstance(params["phase"]["control_dt"], float)
    assert isinstance(params["joints"]["ordered_relevant_joint_names"], list)


def test_dribbling_config():
    config_path = Path(__file__).parent.parent / "configs" / "wolfgang_dribbling_model_config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)

    params = config["kick_node"]["ros__parameters"]

    assert isinstance(params["model"], str)
    assert isinstance(params["phase"]["control_dt"], float)
    assert isinstance(params["joints"]["ordered_relevant_joint_names"], list)
