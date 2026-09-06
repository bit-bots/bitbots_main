# Field maps

Run `pixi run -e default python scripts/field_maps/regenerate.py` with the
`soccer_field_map_generator` package built in the workspace or on `PYTHONPATH`.
The script uses saved generator inputs where available. For legacy fields with
no inputs, it converts the historical intensity encoding while preserving the
raster geometry. The demo field has no localization map.

The generator inputs describe inverted distance maps using the full unsigned
byte intensity range. Keep blackboard geometry overrides when regenerating:
some saved map inputs have different goal depths from the operational configs.
Center-circle diameters come from generator inputs where available; legacy
field diameters were recovered from their map geometry. The demo configuration
has no center circle.

Opponent-set-play positioning reads `field.markings.center_circle.diameter`
from the active field's parameter blackboard. The striker's distance from the
ball is at least the center-circle radius and the configured set-play minimum;
the team separation pass uses the same clearance.
