#[cfg(feature = "gui")]
mod gui {
    use bitbots_rust_nav::map::{ObstacleMap, ObstacleMapConfig};
    use bitbots_rust_nav::obstacle::RoundObstacle;
    use eframe::egui;

    pub struct PathfindingApp {
        config: ObstacleMapConfig,
        obstacles: Vec<RoundObstacle>,
        start: Option<(f64, f64)>,
        goal: Option<(f64, f64)>,
        path: Vec<(f64, f64)>,
        dragging: Option<usize>,
    }

    impl Default for PathfindingApp {
        fn default() -> Self {
            let config = ObstacleMapConfig {
                robot_radius: 0.3,
                margin: 0.25,
                num_vertices: 12,
            };
            let obstacles = vec![
                RoundObstacle {
                    center: (1.0, 4.0),
                    radius: 0.13 / 2.0,
                },
                RoundObstacle {
                    center: (4.2, 4.2),
                    radius: 1.3,
                },
                RoundObstacle {
                    center: (5.0, 2.0),
                    radius: 0.5 / 2.0,
                },
                RoundObstacle {
                    center: (10.0, 10.0),
                    radius: 0.5 / 2.0,
                },
            ];
            Self {
                config,
                obstacles,
                start: None,
                goal: None,
                path: Vec::new(),
                dragging: None,
            }
        }
    }

    impl eframe::App for PathfindingApp {
        fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
            egui::CentralPanel::default().show(ctx, |ui| {
                let (response, painter) = ui
                    .allocate_painter(egui::Vec2::new(500.0, 500.0), egui::Sense::click_and_drag());
                let rect = response.rect;
                let to_screen =
                    |(x, y): (f64, f64)| rect.min + egui::vec2(x as f32 * 40.0, y as f32 * 40.0);

                if let Some(pos) = response.interact_pointer_pos() {
                    let world_pos = (
                        ((pos.x - rect.min.x) / 40.0) as f64,
                        ((pos.y - rect.min.y) / 40.0) as f64,
                    );
                    if response.clicked_by(egui::PointerButton::Primary) {
                        self.start = Some(world_pos);
                    } else if response.clicked_by(egui::PointerButton::Secondary) {
                        self.goal = Some(world_pos);
                    }
                }

                if ctx.input(|i| i.modifiers.shift) {
                    if response.drag_started() {
                        if let Some(pos) = response.interact_pointer_pos() {
                            self.dragging = self.obstacles.iter().position(|o| {
                                let screen_pos = to_screen(o.center);
                                screen_pos.distance(pos)
                                    < (o.radius + self.config.robot_radius + self.config.margin)
                                        as f32
                                        * 40.0
                            });
                        }
                    }

                    if let Some(idx) = self.dragging {
                        if let Some(pos) = response.interact_pointer_pos() {
                            self.obstacles[idx].center = (
                                ((pos.x - rect.min.x) / 40.0) as f64,
                                ((pos.y - rect.min.y) / 40.0) as f64,
                            );
                        }
                    }
                    if response.drag_stopped() {
                        self.dragging = None;
                    }
                }

                for obstacle in &self.obstacles {
                    let pos = to_screen(obstacle.center);
                    let obstacle_layers = vec![
                        (
                            obstacle.radius + self.config.robot_radius + self.config.margin,
                            egui::Color32::YELLOW,
                        ),
                        (
                            obstacle.radius + self.config.robot_radius,
                            egui::Color32::RED,
                        ),
                        (obstacle.radius, egui::Color32::BLACK),
                    ];
                    for (radius, color) in obstacle_layers {
                        painter.circle_filled(pos, (radius as f32) * 40.0, color);
                    }
                }

                if let (Some(start), Some(goal)) = (self.start, self.goal) {
                    let omap = ObstacleMap::new(self.config, self.obstacles.clone());
                    let time1 = std::time::Instant::now();
                    self.path = omap.shortest_path(start, goal);
                    let time2 = std::time::Instant::now();
                    println!("Pathfinding took: {:?}", time2 - time1);
                    for window in self.path.windows(2) {
                        if let [a, b] = window {
                            painter.line_segment(
                                [to_screen(*a), to_screen(*b)],
                                (2.0, egui::Color32::BLUE),
                            );
                        }
                    }
                }
            });
        }
    }

    pub fn run_gui() -> Result<(), eframe::Error> {
        println!("Usage instructions:");
        println!("  - Left click to set the start position");
        println!("  - Right click to set the goal position");
        println!("  - Hold Shift to drag obstacles");
        println!("  - Have fun & Find bugs!");

        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([500.0, 500.0])
                .with_resizable(false),
            ..Default::default()
        };
        eframe::run_native(
            "Interactive Pathfinding",
            options,
            Box::new(|_| Ok(Box::new(PathfindingApp::default()))),
        )
    }
}

#[cfg(feature = "gui")]
fn main() -> Result<(), eframe::Error> {
    gui::run_gui()
}

#[cfg(not(feature = "gui"))]
fn main() {
    println!("GUI feature is disabled. Run with --features gui to enable.");
}
