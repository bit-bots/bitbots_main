use geo::{BooleanOps, Coord, LineString, MultiPolygon, Polygon};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{
    obstacle::{AnyObstacle, Obstacle},
    planner::PathPlanner,
};

/// Configuration values for the ObstacleMap, these should be given by ROS parameters
#[pyclass(
    eq,
    from_py_object,
    str = "ObstacleMapConfig(robot_radius={robot_radius:?}, margin={margin:?}, num_vertices={num_vertices:?})"
)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ObstacleMapConfig {
    /// All obstacles are inflated by this amount, so we can assume the robot itself is a point
    #[pyo3(set, get)]
    pub robot_radius: f64,
    /// Normal plans should not be directly on the edge of an obstacle,
    /// this margin is added if possible
    #[pyo3(set, get)]
    pub margin: f64,
    /// The number of vertices that the polygon representing an obstacle has
    #[pyo3(set, get)]
    pub num_vertices: usize,
}

#[pymethods]
impl ObstacleMapConfig {
    #[new]
    /// Create a new config with the given values
    pub fn new(robot_radius: f64, margin: f64, num_vertices: usize) -> Self {
        Self {
            robot_radius,
            margin,
            num_vertices,
        }
    }
}

/// Hard limits for the robot position in the planning frame, forming an axis-aligned rectangle.
/// In contrast to obstacles, these limits are never crossed by a planned path,
/// not even as a best-effort fallback. A missing limit means the coordinate
/// is unbounded in that direction.
#[pyclass(
    eq,
    from_py_object,
    str = "NavigationBounds(x_min={x_min:?}, x_max={x_max:?}, y_min={y_min:?}, y_max={y_max:?})"
)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct NavigationBounds {
    /// The smallest allowed x coordinate
    #[pyo3(get)]
    pub x_min: Option<f64>,
    /// The largest allowed x coordinate
    #[pyo3(get)]
    pub x_max: Option<f64>,
    /// The smallest allowed y coordinate
    #[pyo3(get)]
    pub y_min: Option<f64>,
    /// The largest allowed y coordinate
    #[pyo3(get)]
    pub y_max: Option<f64>,
}

/// Clamp a single coordinate into the optional limits
fn clamp_coordinate(value: f64, min: Option<f64>, max: Option<f64>) -> f64 {
    let value = min.map_or(value, |min| value.max(min));
    max.map_or(value, |max| value.min(max))
}

/// Check whether a single coordinate lies inside the optional limits (the limits themselves are inside)
fn coordinate_inside(value: f64, min: Option<f64>, max: Option<f64>) -> bool {
    min.is_none_or(|min| value >= min) && max.is_none_or(|max| value <= max)
}

#[pymethods]
impl NavigationBounds {
    #[new]
    #[pyo3(signature = (x_min=None, x_max=None, y_min=None, y_max=None))]
    /// Create new bounds, raises a ValueError if they do not describe a valid region
    pub fn py_new(
        x_min: Option<f64>,
        x_max: Option<f64>,
        y_min: Option<f64>,
        y_max: Option<f64>,
    ) -> PyResult<Self> {
        Self::try_new(x_min, x_max, y_min, y_max).map_err(PyValueError::new_err)
    }

    /// Check whether the given point lies inside the bounds (the limits themselves are inside)
    pub fn contains(&self, point: (f64, f64)) -> bool {
        coordinate_inside(point.0, self.x_min, self.x_max)
            && coordinate_inside(point.1, self.y_min, self.y_max)
    }

    /// Return the closest point inside the bounds
    pub fn clamp(&self, point: (f64, f64)) -> (f64, f64) {
        (
            clamp_coordinate(point.0, self.x_min, self.x_max),
            clamp_coordinate(point.1, self.y_min, self.y_max),
        )
    }
}

impl NavigationBounds {
    /// Create new bounds, fails if a limit is not finite or the limits leave no allowed region
    pub fn try_new(
        x_min: Option<f64>,
        x_max: Option<f64>,
        y_min: Option<f64>,
        y_max: Option<f64>,
    ) -> Result<Self, String> {
        for (axis, min, max) in [("x", x_min, x_max), ("y", y_min, y_max)] {
            for (name, limit) in [("min", min), ("max", max)] {
                if limit.is_some_and(|value| !value.is_finite()) {
                    return Err(format!(
                        "Navigation bound {axis}_{name} must be finite, got {limit:?}"
                    ));
                }
            }
            if let (Some(min), Some(max)) = (min, max) {
                if min >= max {
                    return Err(format!(
                        "Navigation bound {axis}_min ({min}) must be smaller than {axis}_max ({max})"
                    ));
                }
            }
        }
        Ok(Self {
            x_min,
            x_max,
            y_min,
            y_max,
        })
    }
}

#[pyclass(
    eq,
    from_py_object,
    str = "ObstacleMap(obstacles={obstacles:?}, bounds={bounds:?})"
)]
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleMap {
    #[pyo3(get, set)]
    config: ObstacleMapConfig,
    #[pyo3(get, set)]
    obstacles: Vec<AnyObstacle>,
    /// Optional hard limits the planned path never leaves
    #[pyo3(get, set)]
    pub bounds: Option<NavigationBounds>,
}

#[pymethods]
impl ObstacleMap {
    #[new]
    #[pyo3(signature = (config, obstacles, bounds=None))]
    pub fn new(
        config: ObstacleMapConfig,
        obstacles: Vec<AnyObstacle>,
        bounds: Option<NavigationBounds>,
    ) -> Self {
        Self {
            config,
            obstacles,
            bounds,
        }
    }

    pub fn shortest_path(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        PathPlanner::new(self, start, goal).shortest_path()
    }
}

impl ObstacleMap {
    pub fn as_vertices(&self) -> Vec<Coord> {
        self.obstacles
            .iter()
            .flat_map(|obstacle| obstacle.as_vertices(&self.config))
            .collect()
    }

    fn as_polygons(&self, margin: bool) -> Vec<Polygon> {
        self.obstacles
            .iter()
            .map(|obstacle| obstacle.as_polygon(&self.config, margin))
            .collect()
    }

    pub fn as_multipolygon(&self, margin: bool) -> MultiPolygon {
        let polygons = self.as_polygons(margin);
        polygons.iter().fold(
            MultiPolygon::new(vec![Polygon::new(LineString::new(vec![]), vec![])]),
            |a, b| a.union(b),
        )
    }
}
