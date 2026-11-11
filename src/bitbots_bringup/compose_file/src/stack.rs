//! Validated stack and its resolution into concrete commands.

use std::path::Path;

use anyhow::{bail, Context, Result};
use indexmap::IndexMap;

use crate::spec::{ChoicesFrom, ComponentSpec, ParamType, ProcessSpec, StackFile, Value};

/// Current value of every parameter, in declaration order.
pub type Values = IndexMap<String, Value>;

/// Placeholder for a ROS parameter file with overrides of global parameters, provided by
/// whoever runs the stack instead of being declared as parameter.
pub const GLOBAL_PARAMETERS_FILE: &str = "global_parameters_file";

#[derive(Clone, Debug)]
pub struct Parameter {
    pub name: String,
    pub param_type: ParamType,
    pub default: Value,
    pub description: String,
    pub choices_from: Option<ChoicesFrom>,
}

#[derive(Clone, Debug)]
pub struct Profile {
    pub name: String,
    pub description: Option<String>,
    parameters: Values,
    enable: Vec<String>,
    disable: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct Component {
    pub key: String,
    pub name: String,
    pub description: Option<String>,
    pub enabled: bool,
    pub locked: bool,
    pub condition: Values,
    pub requires: Vec<String>,
    pub fallback_for: Option<String>,
    pub processes: Vec<ProcessSpec>,
}

impl Component {
    /// Whether users may switch the component on and off.
    pub fn toggleable(&self) -> bool {
        !self.locked && self.fallback_for.is_none()
    }
}

#[derive(Clone, Debug)]
pub struct Stack {
    parameters: Vec<Parameter>,
    profiles: Vec<Profile>,
    components: Vec<Component>,
}

/// State of every component for a given set of parameter values and selection.
#[derive(Clone, Debug, PartialEq)]
pub struct Resolved {
    /// In the order of [`Stack::components`]
    pub components: Vec<ResolvedComponent>,
    /// Inconsistencies that prevent starting the stack, e.g. a missing requirement
    pub problems: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResolvedComponent {
    /// The parameter conditions of the component are met
    pub available: bool,
    /// The component should run: available and selected
    pub active: bool,
}

const SUPPORTED_VERSION: u32 = 1;

impl Stack {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("cannot read compose file {}", path.display()))?;
        Self::parse(&text).with_context(|| format!("invalid compose file {}", path.display()))
    }

    pub fn parse(yaml: &str) -> Result<Self> {
        let file: StackFile = serde_norway::from_str(yaml)?;
        Self::from_file(file)
    }

    fn from_file(file: StackFile) -> Result<Self> {
        if file.version != SUPPORTED_VERSION {
            bail!(
                "unsupported version {}, expected {SUPPORTED_VERSION}",
                file.version
            );
        }
        let parameters: Vec<Parameter> = file
            .parameters
            .into_iter()
            .map(|(name, p)| Parameter {
                name,
                param_type: p.param_type,
                default: p.default,
                description: p.description,
                choices_from: p.choices_from,
            })
            .collect();
        let components = file
            .components
            .into_iter()
            .map(|(key, spec)| component_from_spec(key, spec))
            .collect::<Result<Vec<_>>>()?;
        let profiles = file
            .profiles
            .into_iter()
            .map(|(name, p)| Profile {
                name,
                description: p.description,
                parameters: p.parameters,
                enable: p.enable,
                disable: p.disable,
            })
            .collect();
        let stack = Stack {
            parameters,
            profiles,
            components,
        };
        let errors = stack.validate();
        if !errors.is_empty() {
            bail!("{}", errors.join("\n"));
        }
        Ok(stack)
    }

    /// All consistency errors of the stack, empty if it is valid.
    fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();
        // Names end up in command lines and URLs.
        let names = self
            .parameters
            .iter()
            .map(|p| ("parameter", &p.name))
            .chain(self.profiles.iter().map(|p| ("profile", &p.name)))
            .chain(self.components.iter().map(|c| ("component", &c.key)));
        for (kind, name) in names {
            if name.is_empty()
                || !name
                    .chars()
                    .all(|ch| ch.is_ascii_lowercase() || ch.is_ascii_digit() || ch == '_')
            {
                errors.push(format!(
                    "{kind} '{name}': names may only contain lowercase letters, digits and '_'"
                ));
            }
        }
        for p in &self.parameters {
            if p.default.param_type() != p.param_type {
                errors.push(format!(
                    "parameter '{}': default does not match type {:?}",
                    p.name, p.param_type
                ));
            }
        }
        let check_values = |owner: &str, values: &Values, errors: &mut Vec<String>| {
            for (name, value) in values {
                if let Err(e) = self.check_value(name, value) {
                    errors.push(format!("{owner}: {e}"));
                }
            }
        };
        for profile in &self.profiles {
            let owner = format!("profile '{}'", profile.name);
            check_values(&owner, &profile.parameters, &mut errors);
            for key in profile.enable.iter().chain(&profile.disable) {
                match self.component(key) {
                    None => errors.push(format!("{owner}: unknown component '{key}'")),
                    Some(c) if !c.toggleable() => errors.push(format!(
                        "{owner}: component '{key}' is locked or a fallback and cannot be switched"
                    )),
                    Some(_) => {}
                }
            }
        }
        for c in &self.components {
            let owner = format!("component '{}'", c.key);
            check_values(&owner, &c.condition, &mut errors);
            for req in &c.requires {
                if req == &c.key {
                    errors.push(format!("{owner}: requires itself"));
                } else if self.component(req).is_none() {
                    errors.push(format!("{owner}: requires unknown component '{req}'"));
                }
            }
            if let Some(target) = &c.fallback_for {
                match self.component(target) {
                    None => errors.push(format!(
                        "{owner}: fallback for unknown component '{target}'"
                    )),
                    Some(t) if t.key == c.key => {
                        errors.push(format!("{owner}: fallback for itself"))
                    }
                    Some(t) if t.fallback_for.is_some() => errors.push(format!(
                        "{owner}: fallback for '{target}', which is a fallback itself"
                    )),
                    Some(_) => {}
                }
                if c.locked {
                    errors.push(format!("{owner}: a fallback cannot be locked"));
                }
            }
            for text in process_strings(&c.processes) {
                for name in placeholders(text) {
                    match name {
                        Ok(name)
                            if self.parameter(name).is_none() && name != GLOBAL_PARAMETERS_FILE =>
                        {
                            errors.push(format!(
                                "{owner}: '${{{name}}}' refers to an unknown parameter"
                            ))
                        }
                        Ok(_) => {}
                        Err(e) => errors.push(format!("{owner}: {e} in '{text}'")),
                    }
                }
            }
        }
        for p in &self.parameters {
            if p.choices_from.is_some() && p.param_type != ParamType::String {
                errors.push(format!(
                    "parameter '{}': choices_from requires type string",
                    p.name
                ));
            }
        }
        if let Some(cycle) = self.requires_cycle() {
            errors.push(format!("requirement cycle: {}", cycle.join(" → ")));
        }
        errors
    }

    /// A cycle in the `requires` graph, if there is one.
    fn requires_cycle(&self) -> Option<Vec<String>> {
        fn visit<'a>(
            stack: &'a Stack,
            key: &'a str,
            path: &mut Vec<&'a str>,
            done: &mut Vec<&'a str>,
        ) -> Option<Vec<String>> {
            if let Some(start) = path.iter().position(|k| *k == key) {
                let mut cycle: Vec<String> = path[start..].iter().map(|k| k.to_string()).collect();
                cycle.push(key.to_string());
                return Some(cycle);
            }
            if done.contains(&key) {
                return None;
            }
            path.push(key);
            for req in stack
                .component(key)
                .map(|c| c.requires.as_slice())
                .unwrap_or(&[])
            {
                if let Some(cycle) = visit(stack, req, path, done) {
                    return Some(cycle);
                }
            }
            path.pop();
            done.push(key);
            None
        }
        let mut done = Vec::new();
        self.components
            .iter()
            .find_map(|c| visit(self, &c.key, &mut Vec::new(), &mut done))
    }

    pub fn parameters(&self) -> &[Parameter] {
        &self.parameters
    }

    pub fn profiles(&self) -> &[Profile] {
        &self.profiles
    }

    pub fn components(&self) -> &[Component] {
        &self.components
    }

    pub fn parameter(&self, name: &str) -> Option<&Parameter> {
        self.parameters.iter().find(|p| p.name == name)
    }

    pub fn component(&self, key: &str) -> Option<&Component> {
        self.components.iter().find(|c| c.key == key)
    }

    pub fn component_idx(&self, key: &str) -> Option<usize> {
        self.components.iter().position(|c| c.key == key)
    }

    pub fn default_values(&self) -> Values {
        self.parameters
            .iter()
            .map(|p| (p.name.clone(), p.default.clone()))
            .collect()
    }

    /// Default selection, in the order of [`Stack::components`].
    pub fn default_enabled(&self) -> Vec<bool> {
        self.components.iter().map(|c| c.enabled).collect()
    }

    /// Fails if `name` is no parameter or `value` does not have its type.
    pub fn check_value(&self, name: &str, value: &Value) -> Result<()> {
        let Some(p) = self.parameter(name) else {
            bail!("unknown parameter '{name}'");
        };
        if value.param_type() != p.param_type {
            bail!(
                "parameter '{name}' expects a {:?} value, got '{value}'",
                p.param_type
            );
        }
        Ok(())
    }

    /// Parse a value given as text, e.g. on the command line, according to the parameter type.
    pub fn parse_value(&self, name: &str, raw: &str) -> Result<Value> {
        let Some(p) = self.parameter(name) else {
            bail!("unknown parameter '{name}'");
        };
        match p.param_type {
            ParamType::Bool => match raw {
                "true" => Ok(Value::Bool(true)),
                "false" => Ok(Value::Bool(false)),
                _ => bail!("parameter '{name}' expects true or false, got '{raw}'"),
            },
            ParamType::String => Ok(Value::String(raw.to_string())),
        }
    }

    /// Apply the parameter values and component selection of a profile.
    pub fn apply_profile(
        &self,
        name: &str,
        values: &mut Values,
        enabled: &mut [bool],
    ) -> Result<()> {
        let Some(profile) = self.profiles.iter().find(|p| p.name == name) else {
            let names: Vec<&str> = self.profiles.iter().map(|p| p.name.as_str()).collect();
            bail!("unknown profile '{name}', available: {}", names.join(", "));
        };
        for (param, value) in &profile.parameters {
            values.insert(param.clone(), value.clone());
        }
        for (keys, on) in [(&profile.enable, true), (&profile.disable, false)] {
            for key in keys {
                let idx = self.component_idx(key).expect("validated on load");
                enabled[idx] = on;
            }
        }
        Ok(())
    }

    /// Availability and activity of all components for the given values and selection.
    pub fn resolve(&self, values: &Values, enabled: &[bool]) -> Resolved {
        assert_eq!(
            enabled.len(),
            self.components.len(),
            "one selection per component"
        );
        let available: Vec<bool> = self
            .components
            .iter()
            .map(|c| {
                c.condition
                    .iter()
                    .all(|(name, v)| values.get(name) == Some(v))
            })
            .collect();
        let selected = |idx: usize| self.components[idx].locked || enabled[idx];
        let mut components: Vec<ResolvedComponent> = (0..self.components.len())
            .map(|i| ResolvedComponent {
                available: available[i],
                active: available[i] && selected(i),
            })
            .collect();
        // Fallbacks depend on the activity of their target, which is never a fallback itself.
        for (i, c) in self.components.iter().enumerate() {
            if let Some(target) = &c.fallback_for {
                let target = self.component_idx(target).expect("validated on load");
                components[i].active = available[i] && !components[target].active;
            }
        }

        let mut problems = Vec::new();
        for (i, c) in self.components.iter().enumerate() {
            if !components[i].active {
                continue;
            }
            for req in &c.requires {
                let r = self.component_idx(req).expect("validated on load");
                if !components[r].active {
                    let why = if components[r].available {
                        "disabled"
                    } else {
                        "not available with the current parameters"
                    };
                    problems.push(format!("{} requires {req}, which is {why}", c.key));
                }
            }
        }
        Resolved {
            components,
            problems,
        }
    }

    /// The commands to start a component with, parameters substituted. `global_parameters_file`
    /// is the path to substitute for [`GLOBAL_PARAMETERS_FILE`].
    pub fn commands(
        &self,
        idx: usize,
        values: &Values,
        global_parameters_file: &str,
    ) -> Vec<Vec<String>> {
        let mut values = values.clone();
        values.insert(
            GLOBAL_PARAMETERS_FILE.to_string(),
            Value::String(global_parameters_file.to_string()),
        );
        let subst = |text: &str| substitute(text, &values);
        self.components[idx]
            .processes
            .iter()
            .map(|p| match p {
                ProcessSpec::Launch(l) => {
                    let mut cmd = vec![
                        "ros2".into(),
                        "launch".into(),
                        l.package.clone(),
                        l.file.clone(),
                    ];
                    cmd.extend(l.args.iter().map(|(k, v)| format!("{k}:={}", subst(v))));
                    cmd
                }
                ProcessSpec::Run(r) => {
                    let mut cmd = vec![
                        "ros2".into(),
                        "run".into(),
                        r.package.clone(),
                        r.executable.clone(),
                    ];
                    cmd.extend(r.args.iter().map(|a| subst(a)));
                    cmd
                }
                ProcessSpec::Command(c) => c.iter().map(|a| subst(a)).collect(),
            })
            .collect()
    }
}

fn component_from_spec(key: String, spec: ComponentSpec) -> Result<Component> {
    let mut processes = Vec::new();
    processes.extend(spec.launch.map(ProcessSpec::Launch));
    processes.extend(spec.run.map(ProcessSpec::Run));
    processes.extend(spec.command.map(ProcessSpec::Command));
    let forms = processes.len() + usize::from(spec.processes.is_some());
    if forms != 1 {
        bail!("component '{key}': give exactly one of launch, run, command or processes");
    }
    if let Some(list) = spec.processes {
        if list.is_empty() {
            bail!("component '{key}': processes must not be empty");
        }
        processes = list;
    }
    for p in &processes {
        if let ProcessSpec::Command(c) = p {
            if c.is_empty() {
                bail!("component '{key}': command must not be empty");
            }
        }
    }
    Ok(Component {
        key,
        name: spec.name,
        description: spec.description,
        enabled: spec.enabled,
        locked: spec.locked,
        condition: spec.condition,
        requires: spec.requires,
        fallback_for: spec.fallback_for,
        processes,
    })
}

/// All strings of the processes that may contain placeholders.
fn process_strings(processes: &[ProcessSpec]) -> Vec<&str> {
    processes
        .iter()
        .flat_map(|p| -> Vec<&str> {
            match p {
                ProcessSpec::Launch(l) => l.args.values().map(String::as_str).collect(),
                ProcessSpec::Run(r) => r.args.iter().map(String::as_str).collect(),
                ProcessSpec::Command(c) => c.iter().map(String::as_str).collect(),
            }
        })
        .collect()
}

/// Names of the `${name}` placeholders in `text`.
fn placeholders(text: &str) -> Vec<Result<&str, String>> {
    let mut found = Vec::new();
    let mut rest = text;
    while let Some(start) = rest.find("${") {
        let after = &rest[start + 2..];
        match after.find('}') {
            Some(end) => {
                found.push(Ok(&after[..end]));
                rest = &after[end + 1..];
            }
            None => {
                found.push(Err("unterminated '${'".to_string()));
                break;
            }
        }
    }
    found
}

fn substitute(text: &str, values: &Values) -> String {
    let mut out = String::new();
    let mut rest = text;
    while let Some(start) = rest.find("${") {
        out.push_str(&rest[..start]);
        let after = &rest[start + 2..];
        let end = after.find('}').expect("validated on load");
        let name = &after[..end];
        let value = values
            .get(name)
            .unwrap_or_else(|| panic!("no value for parameter '{name}'"));
        out.push_str(&value.to_string());
        rest = &after[end + 1..];
    }
    out.push_str(rest);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &str = r#"
version: 1
parameters:
  sim: { type: bool, default: false, description: Simulation }
  field: { type: string, default: lab, description: Field }
profiles:
  sim:
    parameters: { sim: true, field: kid }
    disable: [audio]
components:
  base:
    name: Base
    locked: true
    run: { package: base_pkg, executable: base, args: ["--field", "${field}", "--params-file", "${global_parameters_file}"] }
  hardware:
    name: Hardware
    if: { sim: false }
    command: [driver, "--sim=${sim}"]
  loc:
    name: Localization
    requires: [base]
    launch: { package: loc_pkg, file: loc.launch, args: { sim: "${sim}" } }
  fake_loc:
    name: Fake localization
    fallback_for: loc
    command: [fake]
  audio:
    name: Audio
    processes:
      - command: [mic]
      - launch: { package: audio_pkg, file: audio.launch }
"#;

    fn example() -> Stack {
        Stack::parse(EXAMPLE).unwrap()
    }

    fn active(stack: &Stack, resolved: &Resolved) -> Vec<String> {
        stack
            .components()
            .iter()
            .zip(&resolved.components)
            .filter(|(_, r)| r.active)
            .map(|(c, _)| c.key.clone())
            .collect()
    }

    #[test]
    fn resolves_conditions_fallbacks_and_profiles() {
        let stack = example();
        let mut values = stack.default_values();
        let mut enabled = stack.default_enabled();
        let resolved = stack.resolve(&values, &enabled);
        assert_eq!(
            active(&stack, &resolved),
            ["base", "hardware", "loc", "audio"]
        );
        assert!(resolved.problems.is_empty());

        stack
            .apply_profile("sim", &mut values, &mut enabled)
            .unwrap();
        enabled[stack.component_idx("loc").unwrap()] = false;
        let resolved = stack.resolve(&values, &enabled);
        assert_eq!(active(&stack, &resolved), ["base", "fake_loc"]);
        assert!(!resolved.components[1].available);
    }

    #[test]
    fn missing_requirement_is_a_problem() {
        let stack = example();
        let values = stack.default_values();
        let mut enabled = stack.default_enabled();
        enabled[stack.component_idx("base").unwrap()] = false;
        // Locked components cannot be deselected.
        assert!(stack.resolve(&values, &enabled).problems.is_empty());

        let stack = Stack::parse(&EXAMPLE.replace("    locked: true\n", "")).unwrap();
        let problems = stack.resolve(&values, &enabled).problems;
        assert_eq!(problems, ["loc requires base, which is disabled"]);
    }

    #[test]
    fn commands_substitute_parameters() {
        let stack = example();
        let values = stack.default_values();
        let cmd = |key| stack.commands(stack.component_idx(key).unwrap(), &values, "/g.yaml");
        assert_eq!(
            cmd("base"),
            [[
                "ros2",
                "run",
                "base_pkg",
                "base",
                "--field",
                "lab",
                "--params-file",
                "/g.yaml"
            ]]
        );
        assert_eq!(cmd("hardware"), [["driver", "--sim=false"]]);
        assert_eq!(
            cmd("loc"),
            [["ros2", "launch", "loc_pkg", "loc.launch", "sim:=false"]]
        );
        assert_eq!(cmd("audio").len(), 2);
    }

    #[test]
    fn parse_value_follows_the_declared_type() {
        let stack = example();
        assert_eq!(stack.parse_value("sim", "true").unwrap(), Value::Bool(true));
        assert!(stack.parse_value("sim", "yes").is_err());
        assert_eq!(
            stack.parse_value("field", "true").unwrap(),
            Value::String("true".into())
        );
        assert!(stack.parse_value("nope", "x").is_err());
    }

    fn invalid(replace: (&str, &str)) -> String {
        let yaml = EXAMPLE.replace(replace.0, replace.1);
        assert_ne!(yaml, EXAMPLE, "replacement {replace:?} did not apply");
        format!("{:#}", Stack::parse(&yaml).unwrap_err())
    }

    #[test]
    fn rejects_inconsistent_files() {
        let cases = [
            (("version: 1", "version: 2"), "unsupported version"),
            (
                ("    name: Base\n", "    name: Base\n    nme: typo\n"),
                "unknown field",
            ),
            (("default: lab", "default: true"), "default does not match"),
            (
                ("disable: [audio]", "disable: [nope]"),
                "unknown component 'nope'",
            ),
            (
                ("disable: [audio]", "disable: [base]"),
                "locked or a fallback",
            ),
            (("field: kid", "field: true"), "expects a String"),
            (
                ("if: { sim: false }", "if: { simm: false }"),
                "unknown parameter 'simm'",
            ),
            (
                ("requires: [base]", "requires: [nope]"),
                "unknown component 'nope'",
            ),
            (
                ("fallback_for: loc", "fallback_for: nope"),
                "unknown component 'nope'",
            ),
            (("${field}", "${feld}"), "unknown parameter"),
            (("${field}", "${field"), "unterminated"),
            (("    command: [fake]\n", ""), "exactly one of"),
            (
                (
                    "    command: [fake]\n",
                    "    command: [fake]\n    run: { package: p, executable: e }\n",
                ),
                "exactly one of",
            ),
            (("  base:\n", "  Base:\n"), "lowercase"),
        ];
        for (replace, expected) in cases {
            let err = invalid(replace);
            assert!(
                err.contains(expected),
                "{replace:?}: expected '{expected}' in: {err}"
            );
        }
    }

    #[test]
    fn rejects_requirement_cycles() {
        let yaml = EXAMPLE.replace(
            "    locked: true\n",
            "    locked: true\n    requires: [loc]\n",
        );
        let err = format!("{:#}", Stack::parse(&yaml).unwrap_err());
        assert!(
            err.contains("requirement cycle: base → loc → base"),
            "{err}"
        );
    }
}
