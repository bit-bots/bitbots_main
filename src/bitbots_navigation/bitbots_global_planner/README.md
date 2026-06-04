# bitbots_rust_nav

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

Represents a set of obstacles on a plane with a given config. This has two fields: `config: ObstacleMapConfig` and `obstacles: [RoundObstacle]`. Can be created with the constructor `ObstacleMap(config, obstacles)`
