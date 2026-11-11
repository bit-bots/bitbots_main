"""Verify configurable intensity bounds across map rendering modes."""

import ast
from pathlib import Path

import numpy as np
import pytest
from soccer_field_map_generator.generator import generate_map_image


def parameters():
    tree = ast.parse(
        (Path(__file__).parents[1] / 'soccer_field_map_generator/gui.py').read_text()
    )
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict) and any(
            isinstance(key, ast.Constant) and key.value == 'map_type'
            for key in node.keys
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


@pytest.mark.parametrize('distance_map', [False, True])
@pytest.mark.parametrize('invert', [False, True])
@pytest.mark.parametrize('bounds', [(0, 255), (0, 100), (30, 210)])
def test_bounds(distance_map, invert, bounds):
    config = parameters()
    config.update(distance_map=distance_map, invert=invert)
    original = generate_map_image(config)
    config.update(grayscale_min=bounds[0], grayscale_max=bounds[1])
    result = generate_map_image(config)
    assert result.dtype == np.uint8
    assert result.min() == bounds[0]
    assert result.max() == bounds[1]
    np.testing.assert_array_equal(
        result,
        np.rint(original.astype(float) * (bounds[1] - bounds[0]) / 255 + bounds[0]),
    )


@pytest.mark.parametrize(
    'bounds', [(-1, 255), (0, 256), (100, 100), (200, 100), (0.5, 255)]
)
def test_invalid_bounds(bounds):
    config = parameters()
    config.update(grayscale_min=bounds[0], grayscale_max=bounds[1])
    with pytest.raises(ValueError):
        generate_map_image(config)
