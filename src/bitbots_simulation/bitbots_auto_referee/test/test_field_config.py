"""Field-file radius conversion and automatic set-play distance defaults."""

from pathlib import Path

import pytest

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig


def config(**changes):
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values.update(changes)
    return RefereeConfig.from_parameters(values)


def test_default_distance_is_center_circle_radius():
    assert config().set_play_min_distance == 0.75
    assert config(center_circle_radius=1.2).set_play_min_distance == 1.2


@pytest.mark.parametrize(
    "field_file, radius",
    [
        ("small_division_2026/gui-config.yaml", 0.75),
        ("middle_division_2026/gui-config.yaml", 1.5),
        ("hsl_kid/hsl_kid.yaml", 0.75),
    ],
)
def test_existing_field_files_supply_radius_and_distance(field_file, radius):
    path = Path(__file__).resolve().parents[3] / "bitbots_misc/bitbots_parameter_blackboard/config/fields" / field_file
    configured = config(field_config=str(path))
    assert configured.center_circle_radius == radius
    assert configured.set_play_min_distance == radius


def test_explicit_distance_overrides_automatic_field_radius(tmp_path):
    path = tmp_path / "field.yaml"
    path.write_text("parameters:\n  center_circle_diameter: 240\n")
    configured = config(field_config=str(path), set_play_min_distance=0.9)
    assert configured.center_circle_radius == 1.2
    assert configured.set_play_min_distance == 0.9


@pytest.mark.parametrize("diameter", ["0", "-1", ".nan", ".inf", "true", "invalid"])
def test_invalid_field_diameter_is_rejected(tmp_path, diameter):
    path = tmp_path / "field.yaml"
    path.write_text(f"parameters:\n  center_circle_diameter: {diameter}\n")
    with pytest.raises(ValueError, match="center_circle_diameter"):
        config(field_config=str(path))


@pytest.mark.parametrize("content", ["", "parameters: {}", "parameters: [", "[]"])
def test_missing_or_malformed_field_geometry_is_rejected(tmp_path, content):
    path = tmp_path / "field.yaml"
    path.write_text(content)
    with pytest.raises(ValueError, match="Cannot read center-circle diameter"):
        config(field_config=str(path))


def test_missing_field_file_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="Cannot read center-circle diameter"):
        config(field_config=str(tmp_path / "missing.yaml"))
