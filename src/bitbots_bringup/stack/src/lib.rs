//! Declarative description of a software stack: its parameters, profiles and components.
//!
//! A stack file declares *what* runs and how the parts relate; *how* each part runs is left
//! to ROS launch files and executables. [`Stack::resolve`] turns the file plus the current
//! parameter values and component selection into concrete commands, so consumers such as
//! the orchestrator do not need to know what any parameter or component means.
//!
//! See `config/teamplayer.yaml` in `bitbots_bringup` for the format.

mod spec;
mod stack;

pub use spec::{ChoicesFrom, LaunchSpec, ParamType, ProcessSpec, RunSpec, Value};
pub use stack::{
    Component, Parameter, Profile, Resolved, ResolvedComponent, Stack, Values,
    GLOBAL_PARAMETERS_FILE,
};
