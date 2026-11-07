# bitbots_global_planner

A best-effort A*-on-visibility-graph implementation for path planning in obstacles maps where obstacles and the robot itself are assumed to be round.

## Installation

```bash
pip install .
```

## API

It exports three classes

### `RoundObstacle`

Represents a round obstacle with public fields `center: (float, float)` and `radius: float`. Can be created with the constructor `RoundObstacle(center, radius)`

### `ObstacleMapConfig`

Represents all configuration values for how obstacles should be treated with public fields `dilate: float` - the value by which the radii of the obstacles should be dilated (this should be set to approximately your own radius) - and `num_vertices` - the number of vertices the polygons approximating the round obstacles should have. Can be created with the constructor `ObstacleMapConfig(dilate, num_vertices)`

### `ObstacleMap`

Represents a set of obstacles on a plane with a given config. This has three fields: `config: ObstacleMapConfig`, `obstacles: [RoundObstacle | PolygonObstacle]` and the optional `bounds: NavigationBounds | None`. Can be created with the constructor `ObstacleMap(config, obstacles, bounds=None)`

### `NavigationBounds`

Hard limits for the planned positions, forming an axis-aligned rectangle, with the optional fields `x_min`, `x_max`, `y_min` and `y_max` (each `float | None`, a missing limit means unbounded in that direction). Unlike obstacles, which may be crossed as a best effort fallback, a planned path never leaves these bounds: goals outside of them are clamped onto the closest allowed position. Only the start may lie outside, in which case the path leads back into the bounds. Can be created with the constructor `NavigationBounds(x_min=None, x_max=None, y_min=None, y_max=None)`, which raises a `ValueError` for non-finite limits or a minimum that is not smaller than its maximum. `contains(point)` and `clamp(point)` are exposed as helpers.
