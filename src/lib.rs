use map::{Ball, ObstacleMap, ObstacleMapConfig, Robot};
use pyo3::prelude::*;
pub mod map;
pub mod util;

#[pymodule]
fn bbpprs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Ball>()?;
    m.add_class::<Robot>()?;
    m.add_class::<ObstacleMapConfig>()?;
    m.add_class::<ObstacleMap>()?;
    Ok(())
}
