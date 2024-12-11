use geo::{Coord, LineString, Polygon};

pub fn regular_ngon(center: Coord, radius: f64, sides: usize) -> Polygon {
    let points = (0..sides)
        .map(|side| {
            let angle = (side as f64) / (sides as f64) * std::f64::consts::TAU;
            center + Coord::from((radius * angle.cos(), radius * angle.sin()))
        })
        .collect::<Vec<Coord>>();
    Polygon::new(LineString::new(points.into()), vec![])
}