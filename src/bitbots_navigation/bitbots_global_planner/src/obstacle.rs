use pyo3::prelude::*;

use geo::{BooleanOps, Coord, LineString, MultiPolygon, Polygon};

use crate::map::ObstacleMapConfig;

pub trait Obstacle {
    /// Return the vertices of this obstacle inflated by the robot size and margin for the visibility graph generation
    fn as_vertices(&self, config: &ObstacleMapConfig) -> Vec<Coord>;

    /// Return the polygon representation of this obstacle inflated by the robot size and optionally the margin for intersection checking
    fn as_polygon(&self, config: &ObstacleMapConfig, margin: bool) -> Polygon;
}

/// A helper function to generate a num_vertices-sided polygon approximating a circle
fn regular_ngon(center: Coord, num_vertices: usize, radius: f64) -> Vec<Coord> {
    (0..num_vertices)
        .map(|side| {
            let angle = (side as f64) / (num_vertices as f64) * std::f64::consts::TAU;
            center + Coord::from((radius * angle.cos(), radius * angle.sin()))
        })
        .collect()
}

/// A round obstacle
#[pyclass(eq, from_py_object, str = "RoundObstacle(center={center:?} radius={radius:?})")]
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

/// Value subtracted from the radius of obstacles for intersection checking
pub const EPSILON: f64 = 0.01;

impl Obstacle for RoundObstacle {
    fn as_vertices(&self, config: &ObstacleMapConfig) -> Vec<Coord> {
        regular_ngon(
            self.center.into(),
            config.num_vertices,
            self.radius + config.robot_radius + config.margin,
        )
    }

    fn as_polygon(&self, config: &ObstacleMapConfig, margin: bool) -> Polygon {
        let inflation = if margin {
            config.robot_radius + config.margin
        } else {
            config.robot_radius
        };
        let vertices = regular_ngon(
            self.center.into(),
            config.num_vertices,
            self.radius + inflation - EPSILON,
        );
        Polygon::new(LineString::new(vertices), vec![])
    }
}

/// An obstacle with an arbitrary (also concave) polygon shape
#[pyclass(eq, from_py_object, str = "PolygonObstacle(vertices={vertices:?})")]
#[derive(Debug, Clone, PartialEq)]
pub struct PolygonObstacle {
    /// The vertices of the polygon outline (without a closing duplicate of the first vertex)
    #[pyo3(get, set)]
    pub vertices: Vec<(f64, f64)>,
}

#[pymethods]
impl PolygonObstacle {
    #[new]
    pub fn new(vertices: Vec<(f64, f64)>) -> Self {
        Self { vertices }
    }
}

impl PolygonObstacle {
    /// Inflate the polygon by the given offset (Minkowski sum with a num_vertices-sided
    /// approximation of a circle). This is done by unioning the polygon with a rectangle
    /// for each edge and an ngon for each vertex.
    fn buffered(&self, num_vertices: usize, offset: f64) -> Polygon {
        let coords: Vec<Coord> = self.vertices.iter().copied().map(Coord::from).collect();
        let base = Polygon::new(LineString::new(coords.clone()), vec![]);
        if offset <= 0.0 {
            return base;
        }
        let mut result = MultiPolygon::new(vec![base.clone()]);
        for i in 0..coords.len() {
            let a = coords[i];
            let b = coords[(i + 1) % coords.len()];
            let edge = b - a;
            let length = edge.x.hypot(edge.y);
            if length > 0.0 {
                // Rectangle covering the edge inflated perpendicular to it
                let normal = Coord::from((-edge.y / length * offset, edge.x / length * offset));
                let rectangle = Polygon::new(
                    LineString::new(vec![a + normal, b + normal, b - normal, a - normal]),
                    vec![],
                );
                result = result.union(&rectangle);
            }
            // Regular ngon rounding the corner at each vertex
            let corner = Polygon::new(LineString::new(regular_ngon(a, num_vertices, offset)), vec![]);
            result = result.union(&corner);
        }
        // All parts overlap the base polygon, so the union is a single polygon
        result.0.into_iter().next().unwrap_or(base)
    }
}

impl Obstacle for PolygonObstacle {
    fn as_vertices(&self, config: &ObstacleMapConfig) -> Vec<Coord> {
        let polygon = self.buffered(config.num_vertices, config.robot_radius + config.margin);
        let exterior = polygon.exterior();
        // Skip the closing duplicate of the first vertex
        exterior.0[..exterior.0.len().saturating_sub(1)].to_vec()
    }

    fn as_polygon(&self, config: &ObstacleMapConfig, margin: bool) -> Polygon {
        let inflation = if margin {
            config.robot_radius + config.margin
        } else {
            config.robot_radius
        };
        self.buffered(config.num_vertices, inflation - EPSILON)
    }
}

/// Any of the supported obstacle shapes, so they can be mixed in the same ObstacleMap
#[derive(Debug, Clone, PartialEq, FromPyObject, IntoPyObject)]
pub enum AnyObstacle {
    Round(RoundObstacle),
    Polygon(PolygonObstacle),
}

impl Obstacle for AnyObstacle {
    fn as_vertices(&self, config: &ObstacleMapConfig) -> Vec<Coord> {
        match self {
            AnyObstacle::Round(obstacle) => obstacle.as_vertices(config),
            AnyObstacle::Polygon(obstacle) => obstacle.as_vertices(config),
        }
    }

    fn as_polygon(&self, config: &ObstacleMapConfig, margin: bool) -> Polygon {
        match self {
            AnyObstacle::Round(obstacle) => obstacle.as_polygon(config, margin),
            AnyObstacle::Polygon(obstacle) => obstacle.as_polygon(config, margin),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::{ObstacleMap, ObstacleMapConfig};
    use geo::Contains;

    fn config() -> ObstacleMapConfig {
        ObstacleMapConfig {
            robot_radius: 0.3,
            margin: 0.1,
            num_vertices: 12,
        }
    }

    #[test]
    fn polygon_obstacle_is_inflated() {
        let obstacle =
            PolygonObstacle::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
        // The polygon inflated by the robot radius contains points outside of the original polygon
        let critical = obstacle.as_polygon(&config(), false);
        assert!(critical.contains(&Coord::from((-0.25, 0.5))));
        assert!(!critical.contains(&Coord::from((-0.35, 0.5))));
        // The polygon inflated by the robot radius and margin is even larger
        let with_margin = obstacle.as_polygon(&config(), true);
        assert!(with_margin.contains(&Coord::from((-0.35, 0.5))));
    }

    #[test]
    fn path_avoids_polygon_obstacle() {
        // A wall between the start and the goal
        let wall = PolygonObstacle::new(vec![(2.0, -2.0), (2.2, -2.0), (2.2, 2.0), (2.0, 2.0)]);
        let map = ObstacleMap::new(config(), vec![AnyObstacle::Polygon(wall)]);
        let path = map.shortest_path((0.0, 0.0), (4.0, 0.0));
        // The path needs to route around one of the wall ends instead of going straight
        assert!(path.len() > 2);
        assert!(path.iter().any(|(_, y)| y.abs() > 2.0));
    }
}
