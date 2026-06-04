use pyo3::prelude::*;

use geo::{Coord, LineString, Polygon};

use crate::map::ObstacleMapConfig;

pub trait Obstacle {
    /// Return the vertices of this obstacle inflated by the robot size and margin for the visibility graph generation
    fn as_vertices(&self, config: &ObstacleMapConfig) -> Vec<Coord>;

    /// Return the polygon representation of this obstacle inflated by the robot size and optionally the margin for intersection checking
    fn as_polygon(&self, config: &ObstacleMapConfig, margin: bool) -> Polygon;
}

/// A round obstacle
#[pyclass(eq, str = "RoundObstacle(center={center:?} radius={radius:?})")]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoundObstacle {
    /// The center of the obstacle
    #[pyo3(get, set)]
    pub center: (f64, f64),
    /// The radius of the obstacle
    #[pyo3(get, set)]
    pub radius: f64,
}

#[pymethods]
impl RoundObstacle {
    #[new]
    pub fn new(center: (f64, f64), radius: f64) -> Self {
        Self { center, radius }
    }
}

impl RoundObstacle {
    /// A helper function to generate a num_vertices-sided Polygon with own radius plus offset
    fn regular_ngon(&self, num_vertices: usize, offset: f64) -> Vec<Coord> {
        (0..num_vertices)
            .map(|side| {
                let angle = (side as f64) / (num_vertices as f64) * std::f64::consts::TAU;
                let radius = self.radius + offset;
                Coord::from(self.center) + Coord::from((radius * angle.cos(), radius * angle.sin()))
            })
            .collect()
    }
}

/// Value subtracted from the radius of obstacles for intersection checking
pub const EPSILON: f64 = 0.01;

impl Obstacle for RoundObstacle {
    fn as_vertices(&self, config: &ObstacleMapConfig) -> Vec<Coord> {
        self.regular_ngon(config.num_vertices, config.robot_radius + config.margin)
    }

    fn as_polygon(&self, config: &ObstacleMapConfig, margin: bool) -> Polygon {
        let inflation = if margin {
            config.robot_radius + config.margin
        } else {
            config.robot_radius
        };
        let vertices = self.regular_ngon(config.num_vertices, inflation - EPSILON);
        Polygon::new(LineString::new(vertices), vec![])
    }
}
