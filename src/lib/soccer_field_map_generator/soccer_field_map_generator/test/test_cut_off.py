"""Verify optional horizontal map cropping and its GUI configuration."""

import ast
import io
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from soccer_field_map_generator.generator import (
    drawDistance,
    generate_map_image,
    generate_metadata,
)
import soccer_field_map_generator.gui as gui_module


def parameters():
    tree = ast.parse((Path(__file__).parents[1] / 'soccer_field_map_generator/gui.py').read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict) and any(
            isinstance(key, ast.Constant) and key.value == 'map_type' for key in node.keys
        ):
            result = {}
            for key, definition in zip(node.keys, node.values):
                for name, value in zip(definition.keys, definition.values):
                    if name.value == 'default':
                        result[key.value] = (
                            value.attr.lower()
                            if isinstance(value, ast.Attribute)
                            else ast.literal_eval(value)
                        )
            return result
    raise AssertionError('GUI parameter definitions missing')


@pytest.mark.parametrize(
    'map_type',
    [
        'line',
        'posts',
        'field_boundary',
        'field_features',
        'corners',
        'tcrossings',
        'crosses',
    ],
)
def test_crop_selects_requested_columns(map_type):
    config = parameters()
    config.update(map_type=map_type, distance_map=False, invert=False)
    complete_map = generate_map_image(config)
    config.update(crop_enabled=True, crop_start=1.25, crop_end=2.75)

    masked_map = generate_map_image(config)

    expected_map = complete_map.copy()
    expected_map[:, :125] = 0
    expected_map[:, 275:] = 0
    assert masked_map.shape == complete_map.shape
    np.testing.assert_array_equal(masked_map, expected_map)


def test_fractional_meter_bounds_cover_pixel_cells():
    config = parameters()
    config.update(distance_map=False, invert=False, crop_enabled=False)
    complete_map = generate_map_image(config)
    config.update(crop_enabled=True, crop_start=0.015, crop_end=0.035)

    masked_map = generate_map_image(config)

    expected_map = complete_map.copy()
    expected_map[:, :1] = 0
    expected_map[:, 4:] = 0
    assert masked_map.shape == complete_map.shape
    np.testing.assert_array_equal(masked_map, expected_map)


@pytest.mark.parametrize('invert', [False, True])
def test_feature_removal_precedes_distance_map_and_inversion(invert):
    config = parameters()
    config.update(distance_map=False, invert=False)
    complete_map = generate_map_image(config)
    config.update(crop_enabled=True, crop_start=1.0, crop_end=3.0)
    masked_map = generate_map_image(config)
    expected_features = complete_map.copy()
    expected_features[:, :100] = 0
    expected_features[:, 300:] = 0
    np.testing.assert_array_equal(masked_map, expected_features)

    config.update(distance_map=True, distance_decay=0.5, invert=invert)
    distance_map = generate_map_image(config)
    expected_distance_map = drawDistance(expected_features, 0.5)
    if invert:
        expected_distance_map = 255 - expected_distance_map
    np.testing.assert_array_equal(distance_map, expected_distance_map)
    assert np.unique(distance_map[:, :100]).size > 1


def test_feature_removal_respects_custom_grayscale_bounds():
    config = parameters()
    config.update(distance_map=False, invert=False, grayscale_min=30, grayscale_max=210)
    complete_map = generate_map_image(config)
    config.update(crop_enabled=True, crop_start=1.0, crop_end=3.0)

    masked_map = generate_map_image(config)

    expected_map = complete_map.copy()
    expected_map[:, :100] = 30
    expected_map[:, 300:] = 30
    assert masked_map.shape == complete_map.shape
    np.testing.assert_array_equal(masked_map, expected_map)


def test_distance_map_rejects_crop_without_features():
    config = parameters()
    config.update(
        map_type='posts',
        distance_map=True,
        crop_enabled=True,
        crop_start=3.0,
        crop_end=4.0,
    )

    with pytest.raises(ValueError, match='contains no features'):
        generate_map_image(config)


def test_legacy_parameters_keep_the_full_map():
    config = parameters()
    expected_map = generate_map_image(config)
    config.pop('crop_enabled')
    config.pop('crop_start')
    config.pop('crop_end')

    np.testing.assert_array_equal(generate_map_image(config), expected_map)


@pytest.mark.parametrize(
    ('crop_start', 'crop_end'),
    [
        (-0.01, 1.0),
        (1.0, 0.5),
        (1.0, 1.0),
        (0.0, 11.01),
        (float('nan'), 1.0),
        (0.0, float('inf')),
        ('invalid', 1.0),
        (True, 1.0),
        (10**1000, 1.0),
    ],
)
def test_invalid_crop_range_is_rejected(crop_start, crop_end):
    config = parameters()
    config.update(crop_enabled=True, crop_start=crop_start, crop_end=crop_end)

    with pytest.raises(ValueError):
        generate_map_image(config)


def test_enabled_crop_requires_both_bounds():
    config = parameters()
    config['crop_enabled'] = True
    config.pop('crop_start')

    with pytest.raises(ValueError):
        generate_map_image(config)


@pytest.mark.parametrize('map_type', ['line', 'field_boundary'])
def test_cut_off_does_not_change_map_metadata(map_type):
    config = parameters()
    config.update(map_type=map_type, crop_enabled=False)
    original_metadata = generate_metadata(config, 'map.png')
    config.update(crop_enabled=True, crop_start=2.0, crop_end=4.0)

    cut_off_metadata = generate_metadata(config, 'map.png')

    assert cut_off_metadata == original_metadata


def test_loading_older_config_keeps_new_crop_controls_at_defaults(monkeypatch):
    def make_variable(initial_value):
        value = [initial_value]
        return SimpleNamespace(
            get=lambda: value[0],
            set=lambda new_value: value.__setitem__(0, new_value),
        )

    gui = gui_module.MapGeneratorGUI.__new__(gui_module.MapGeneratorGUI)
    gui.parameter_input = SimpleNamespace(
        parameter_values={
            'invert': make_variable(True),
            'crop_enabled': make_variable(False),
            'crop_start': make_variable(0.0),
            'crop_end': make_variable(10.0),
        }
    )
    gui.update_map = lambda: None
    old_config = io.StringIO(
        "header:\n  version: '1.0'\n  type: map_generator_config\nparameters:\n  invert: false\n"
    )
    monkeypatch.setattr(
        gui_module.filedialog,
        'askopenfile',
        lambda **kwargs: old_config,
    )

    gui.load_config()

    assert gui.parameter_input.parameter_values['invert'].get() is False
    assert gui.parameter_input.parameter_values['crop_enabled'].get() is False
    assert gui.parameter_input.parameter_values['crop_start'].get() == 0.0
    assert gui.parameter_input.parameter_values['crop_end'].get() == 10.0


def test_save_map_reports_invalid_crop_range(monkeypatch):
    config = parameters()
    config.update(crop_enabled=True, crop_start=3.0, crop_end=2.0)
    gui = gui_module.MapGeneratorGUI.__new__(gui_module.MapGeneratorGUI)
    gui.parameter_input = SimpleNamespace(get_parameters=lambda: config)
    dialog_calls = []
    errors = []
    monkeypatch.setattr(
        gui_module.filedialog,
        'asksaveasfile',
        lambda **kwargs: dialog_calls.append(kwargs) or SimpleNamespace(name='map.png'),
    )
    monkeypatch.setattr(
        gui_module.tk,
        'messagebox',
        SimpleNamespace(showerror=lambda *args: errors.append(args)),
        raising=False,
    )

    gui.save_map()

    assert not dialog_calls
    assert len(errors) == 1
