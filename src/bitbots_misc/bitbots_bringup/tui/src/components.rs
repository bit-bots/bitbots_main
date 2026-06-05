// ─── Global params ────────────────────────────────────────────────────────────

pub const FIELDNAME_DEFAULT_HW: &str = "labor";
pub const FIELDNAME_DEFAULT_SIM: &str = "hsl_kid";

#[derive(Clone, Debug)]
pub struct GlobalParams {
    pub sim: bool,
    pub zenoh: bool,
    pub fieldname: String,
    pub dsd_file: String,
}

impl Default for GlobalParams {
    fn default() -> Self {
        Self {
            sim: false,
            zenoh: true,
            fieldname: FIELDNAME_DEFAULT_HW.to_string(),
            dsd_file: "main.dsd".to_string(),
        }
    }
}

// ─── Component definitions ────────────────────────────────────────────────────

pub struct ComponentDef {
    pub name: &'static str,
    pub key: &'static str,
    pub default_enabled: bool,
    pub infrastructure: bool,
    /// Only valid on real hardware — hidden/disabled in sim
    pub hardware_only: bool,
    /// Only shown/started when sim mode is active (e.g. the simulator itself)
    pub sim_component: bool,
    /// Returns the list of commands to spawn for this component
    pub cmds: fn(&GlobalParams) -> Vec<Vec<String>>,
}

// ─── Command helpers ──────────────────────────────────────────────────────────

pub fn ros2_launch(pkg: &str, file: &str, extra: &[&str]) -> Vec<String> {
    let mut v = vec!["ros2".into(), "launch".into(), pkg.into(), file.into()];
    v.extend(extra.iter().map(|s| s.to_string()));
    v
}

pub fn ros2_run(pkg: &str, exec: &str, extra: &[&str]) -> Vec<String> {
    let mut v = vec!["ros2".into(), "run".into(), pkg.into(), exec.into()];
    v.extend(extra.iter().map(|s| s.to_string()));
    v
}

fn sim(p: &GlobalParams) -> String {
    format!("sim:={}", p.sim)
}

// ─── Static component registry ────────────────────────────────────────────────

pub static COMPONENT_DEFS: &[ComponentDef] = &[
    // ── Infrastructure (always started) ──────────────────────────────────────
    ComponentDef {
        name: "Zenoh",
        key: "zenoh",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        sim_component: false,
        cmds: |_| vec![ros2_run("rmw_zenoh_cpp", "rmw_zenohd", &[])],
    },
    ComponentDef {
        name: "Param Blackboard",
        key: "blackboard",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_parameter_blackboard",
                "parameter_blackboard.launch",
                &[&sim(p), &format!("fieldname:={}", p.fieldname)],
            )]
        },
    },
    ComponentDef {
        name: "Robot Description",
        key: "robot_description",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_robot_description",
                "load_robot_description.launch",
                &[&sim(p)],
            )]
        },
    },
    ComponentDef {
        name: "Diagnostics",
        key: "diagnostics",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        sim_component: false,
        cmds: |_| vec![ros2_launch("bitbots_diagnostic", "aggregator.launch", &[])],
    },
    ComponentDef {
        name: "Simulator",
        key: "simulator",
        default_enabled: false,
        infrastructure: true,
        hardware_only: false,
        sim_component: true,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_mujoco_sim",
                "simulator.launch",
                &[&sim(p)],
            )]
        },
    },
    // ── User-toggleable ───────────────────────────────────────────────────────
    ComponentDef {
        name: "Lowlevel",
        key: "lowlevel",
        default_enabled: true,
        infrastructure: false,
        hardware_only: true,
        sim_component: false,
        cmds: |_| vec![ros2_launch("livelybot_bringup", "lowlevel.launch", &[])],
    },
    ComponentDef {
        name: "Motion",
        key: "motion",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_bringup",
                "motion.launch",
                &[&sim(p), "lowlevel:=false"],
            )]
        },
    },
    ComponentDef {
        name: "Game Controller",
        key: "game_controller",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "game_controller_hsl",
                "game_controller.launch",
                &[
                    &sim(p),
                    "use_parameter_blackboard:=true",
                    "parameter_blackboard_name:=parameter_blackboard",
                    "team_id_param_name:=team_id",
                    "bot_id_param_name:=bot_id",
                ],
            )]
        },
    },
    ComponentDef {
        name: "Vision",
        key: "vision",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| vec![ros2_launch("bitbots_bringup", "vision.launch", &[&sim(p)])],
    },
    ComponentDef {
        name: "IPM",
        key: "ipm",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| vec![ros2_launch("bitbots_ipm", "ipm.launch", &[&sim(p)])],
    },
    ComponentDef {
        name: "Localization",
        key: "localization",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_localization",
                "localization.launch",
                &[&sim(p)],
            )]
        },
    },
    ComponentDef {
        name: "Path Planning",
        key: "path_planning",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_path_planning",
                "path_planning.launch",
                &[&sim(p)],
            )]
        },
    },
    ComponentDef {
        name: "Behavior",
        key: "behavior",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_body_behavior",
                "behavior.launch",
                &[&sim(p), &format!("dsd_file:={}", p.dsd_file)],
            )]
        },
    },
    ComponentDef {
        name: "Team Comm",
        key: "teamcom",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_team_communication",
                "team_comm.launch",
                &[&sim(p)],
            )]
        },
    },
    ComponentDef {
        name: "World Model",
        key: "world_model",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![
                ros2_launch("bitbots_ball_filter", "ball_filter.launch", &[&sim(p)]),
                ros2_launch("bitbots_robot_filter", "robot_filter.launch", &[&sim(p)]),
            ]
        },
    },
    ComponentDef {
        name: "Whistle Det.",
        key: "whistle_detector",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |_| {
            vec![ros2_launch(
                "bitbots_whistle_detector",
                "whistle_detector.launch",
                &[],
            )]
        },
    },
    ComponentDef {
        name: "Audio",
        key: "audio",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |_| vec![ros2_launch("bitbots_bringup", "audio.launch", &[])],
    },
    ComponentDef {
        name: "TTS",
        key: "tts",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |_| vec![ros2_launch("bitbots_tts", "tts.launch", &[])],
    },
    ComponentDef {
        name: "Monitoring",
        key: "monitoring",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |_| vec![ros2_launch("bitbots_bringup", "monitoring.launch", &[])],
    },
    ComponentDef {
        name: "Recording",
        key: "record",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        sim_component: false,
        cmds: |p| {
            vec![ros2_launch(
                "bitbots_bringup",
                "rosbag_record.launch.py",
                &[&sim(p)],
            )]
        },
    },
];
