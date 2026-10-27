//! Assembly of the global parameters and field maps declared in the `blackboard` section of the
//! stack file. Publishing them is left to [`crate::ros`].

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use bitbots_stack::{BlackboardSpec, Stack, Value, Values};
use indexmap::IndexMap;

use crate::api::{GlobalSource, GlobalValue};

/// Highest intensity in a field map image, read as occupancy percentage.
const FIELD_MAP_MAX_INTENSITY: u8 = 100;

/// Everything the blackboard publishes.
#[derive(Clone, Debug, PartialEq)]
pub struct Blackboard {
    /// Sorted by name
    pub parameters: IndexMap<String, (GlobalValue, GlobalSource)>,
    pub maps: Vec<FieldMap>,
}

/// A field map as occupancy grid, centered on the field center.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldMap {
    pub topic: String,
    pub frame_id: String,
    pub resolution: f64,
    pub width: u32,
    pub height: u32,
    /// Row major, in the row order of the image
    pub data: Vec<i8>,
}

/// Assemble the blackboard from its files, the stack parameters and the overrides.
///
/// `share_dir` is the share directory of `spec.package`. Overrides must name parameters that
/// exist without them and keep their type.
pub fn assemble(
    stack: &Stack,
    spec: &BlackboardSpec,
    values: &Values,
    overrides: &IndexMap<String, GlobalValue>,
    share_dir: &Path,
) -> Result<Blackboard> {
    let mut parameters = IndexMap::new();
    for file in &spec.files {
        let path = share_dir.join(stack.substitute(file, values));
        for (name, value) in load_parameter_file(&path, &spec.node)? {
            let source = GlobalSource::File(path.display().to_string());
            parameters.insert(name, (value, source));
        }
    }
    for (name, text) in &spec.parameters {
        let value = match stack.substitute_typed(text, values) {
            Value::Bool(b) => GlobalValue::Bool(b),
            Value::String(s) => GlobalValue::String(s),
        };
        parameters.insert(name.clone(), (value, GlobalSource::Stack));
    }
    for (name, value) in overrides {
        let Some((current, _)) = parameters.get(name) else {
            bail!("override of unknown global parameter '{name}'");
        };
        let value = coerce(name, value, current)?;
        parameters.insert(name.clone(), (value, GlobalSource::Override));
    }
    parameters.sort_keys();

    let mut maps = Vec::new();
    if let Some(m) = &spec.maps {
        let frame_id = match parameters.get(&m.frame_parameter) {
            Some((GlobalValue::String(frame), _)) => frame.clone(),
            Some((other, _)) => bail!(
                "map frame parameter '{}' is a {}, expected a string",
                m.frame_parameter,
                other.type_name()
            ),
            None => bail!("map frame parameter '{}' is not set", m.frame_parameter),
        };
        let dir = share_dir.join(stack.substitute(&m.directory, values));
        for (name, image) in &m.images {
            let path = dir.join(image);
            // Not every field provides every map; nodes that need a missing one wait for it.
            if !path.is_file() {
                continue;
            }
            let (width, height, data) = load_field_map(&path)?;
            maps.push(FieldMap {
                topic: format!("{}{name}", m.topic_prefix),
                frame_id: frame_id.clone(),
                resolution: m.resolution,
                width,
                height,
                data,
            });
        }
    }
    Ok(Blackboard { parameters, maps })
}

/// Convert an override to the type of the value it replaces. Integers are accepted for doubles.
pub fn coerce(name: &str, value: &GlobalValue, current: &GlobalValue) -> Result<GlobalValue> {
    match (value, current) {
        (GlobalValue::Integer(i), GlobalValue::Double(_)) => Ok(GlobalValue::Double(*i as f64)),
        (GlobalValue::IntegerArray(a), GlobalValue::DoubleArray(_)) => Ok(
            GlobalValue::DoubleArray(a.iter().map(|i| *i as f64).collect()),
        ),
        (v, c) if std::mem::discriminant(v) == std::mem::discriminant(c) => Ok(v.clone()),
        (v, c) => bail!(
            "global parameter '{name}' is a {}, got a {} ({v})",
            c.type_name(),
            v.type_name()
        ),
    }
}

/// Parse an override given as text, e.g. on the command line, with the type of `current`.
pub fn parse_override(name: &str, raw: &str, current: &GlobalValue) -> Result<GlobalValue> {
    // Strings stay strings even if they look like something else, e.g. a number.
    if let GlobalValue::String(_) = current {
        return Ok(GlobalValue::String(raw.to_string()));
    }
    let value: serde_norway::Value =
        serde_norway::from_str(raw).with_context(|| format!("invalid value for '{name}'"))?;
    coerce(name, &yaml_to_value(name, &value)?, current)
}

/// The parameters of `node` in a ROS parameter file, flattened with `.` separators.
fn load_parameter_file(path: &Path, node: &str) -> Result<Vec<(String, GlobalValue)>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("cannot read global parameter file {}", path.display()))?;
    let doc: serde_norway::Value = serde_norway::from_str(&text)
        .with_context(|| format!("invalid YAML in {}", path.display()))?;
    let mut out = Vec::new();
    let Some(root) = doc.as_mapping() else {
        bail!("{} does not contain a mapping", path.display());
    };
    for (key, section) in root {
        let key = key.as_str().unwrap_or_default();
        let applies = [
            node.to_string(),
            format!("/{node}"),
            "/**".into(),
            "**".into(),
        ];
        if !applies.iter().any(|k| k == key) {
            continue;
        }
        let Some(params) = section.get("ros__parameters") else {
            bail!("{}: section '{key}' has no ros__parameters", path.display());
        };
        flatten(params, String::new(), &mut out)
            .with_context(|| format!("in {}", path.display()))?;
    }
    Ok(out)
}

fn flatten(
    value: &serde_norway::Value,
    prefix: String,
    out: &mut Vec<(String, GlobalValue)>,
) -> Result<()> {
    if let Some(map) = value.as_mapping() {
        for (key, child) in map {
            let Some(key) = key.as_str() else {
                bail!("non-string key below '{prefix}'");
            };
            let name = if prefix.is_empty() {
                key.to_string()
            } else {
                format!("{prefix}.{key}")
            };
            flatten(child, name, out)?;
        }
        return Ok(());
    }
    out.push((prefix.clone(), yaml_to_value(&prefix, value)?));
    Ok(())
}

fn yaml_to_value(name: &str, value: &serde_norway::Value) -> Result<GlobalValue> {
    use serde_norway::Value as Y;
    Ok(match value {
        Y::Bool(b) => GlobalValue::Bool(*b),
        Y::Number(n) if n.is_i64() => GlobalValue::Integer(n.as_i64().expect("checked")),
        Y::Number(n) => GlobalValue::Double(
            n.as_f64()
                .with_context(|| format!("'{name}': number out of range"))?,
        ),
        Y::String(s) => GlobalValue::String(s.clone()),
        Y::Sequence(items) => {
            if items.is_empty() {
                bail!("'{name}': empty arrays have no type, which ROS parameters require");
            }
            let all = |f: fn(&Y) -> bool| items.iter().all(f);
            if all(Y::is_bool) {
                GlobalValue::BoolArray(items.iter().filter_map(Y::as_bool).collect())
            } else if all(Y::is_i64) {
                GlobalValue::IntegerArray(items.iter().filter_map(Y::as_i64).collect())
            } else if all(Y::is_number) {
                GlobalValue::DoubleArray(items.iter().filter_map(Y::as_f64).collect())
            } else if all(Y::is_string) {
                GlobalValue::StringArray(
                    items
                        .iter()
                        .filter_map(|v| v.as_str().map(str::to_string))
                        .collect(),
                )
            } else {
                bail!("'{name}': arrays must contain values of one type");
            }
        }
        other => bail!("'{name}': unsupported value {other:?}"),
    })
}

/// Occupancy values of a field map image: 0 for the brightest allowed intensity, 100 for black.
fn load_field_map(path: &Path) -> Result<(u32, u32, Vec<i8>)> {
    let image = image::open(path)
        .with_context(|| format!("cannot read field map {}", path.display()))?
        .to_luma8();
    let max = image.pixels().map(|p| p.0[0]).max().unwrap_or(0);
    if max > FIELD_MAP_MAX_INTENSITY {
        bail!(
            "field map {} contains intensities up to {max}, expected at most \
             {FIELD_MAP_MAX_INTENSITY}; maps from the field map generator need to be converted",
            path.display()
        );
    }
    let data = image
        .pixels()
        .map(|p| (FIELD_MAP_MAX_INTENSITY - p.0[0]) as i8)
        .collect();
    Ok((image.width(), image.height(), data))
}

/// Names of the subdirectories of `dir`, sorted.
pub fn subdirectories(dir: &Path) -> Result<Vec<String>> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .with_context(|| format!("cannot list {}", dir.display()))?
        .flatten()
        .filter(|e| e.path().is_dir())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    Ok(names)
}

/// Locates the share directories of ROS packages.
pub type ShareResolver = Box<dyn Fn(&str) -> Result<PathBuf> + Send>;

/// Share directory of a package in the sourced workspace, via AMENT_PREFIX_PATH.
pub fn ament_share(package: &str) -> Result<PathBuf> {
    let prefixes = std::env::var_os("AMENT_PREFIX_PATH").unwrap_or_default();
    for prefix in std::env::split_paths(&prefixes) {
        let dir = prefix.join("share").join(package);
        if dir.is_dir() {
            return Ok(dir);
        }
    }
    bail!("package {package} not found in AMENT_PREFIX_PATH; is the workspace built and sourced?")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(dir: &Path, name: &str, text: &str) {
        let path = dir.join(name);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }

    fn temp_dir(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "bitbots_orchestrate_bb_{}_{test}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    const STACK: &str = r#"
version: 1
parameters:
  sim: { type: bool, default: false, description: Sim }
  field: { type: string, default: lab, description: Field }
components:
  router: { name: Router, command: [router] }
blackboard:
  node: board
  topic: global
  package: pkg
  requires: [router]
  files: ["fields/${field}/config.yaml", settings.yaml]
  parameters: { simulation_active: "${sim}", field.name: "${field}" }
  maps:
    directory: fields/${field}
    frame_parameter: map_frame
    resolution: 0.01
    topic_prefix: field/map/
    images: { lines: lines.png, goals: goals.png }
"#;

    fn fixture(test: &str) -> (Stack, PathBuf) {
        let dir = temp_dir(test);
        write(
            &dir,
            "fields/lab/config.yaml",
            "board:\n  ros__parameters:\n    field:\n      size:\n        x: 5.0\n        y: 3\n    bot_id: 9\n",
        );
        write(
            &dir,
            "settings.yaml",
            "/**:\n  ros__parameters:\n    bot_id: 1\n    role: offense\n    map_frame: map\n    ids: [1, 2]\nother_node:\n  ros__parameters:\n    ignored: true\n",
        );
        let mut img = image::GrayImage::new(3, 2);
        img.put_pixel(0, 0, image::Luma([100]));
        img.save(dir.join("fields/lab/lines.png")).unwrap();
        (Stack::parse(STACK).unwrap(), dir)
    }

    fn assemble_with(
        stack: &Stack,
        dir: &Path,
        overrides: &[(&str, GlobalValue)],
    ) -> Result<Blackboard> {
        let overrides = overrides
            .iter()
            .map(|(n, v)| (n.to_string(), v.clone()))
            .collect();
        assemble(
            stack,
            stack.blackboard().unwrap(),
            &stack.default_values(),
            &overrides,
            dir,
        )
    }

    #[test]
    fn files_stack_parameters_and_overrides_take_precedence_in_order() {
        let (stack, dir) = fixture("precedence");
        let bb = assemble_with(
            &stack,
            &dir,
            &[("role", GlobalValue::String("goalie".into()))],
        )
        .unwrap();
        let get = |n: &str| bb.parameters[n].clone();
        assert_eq!(get("field.size.x").0, GlobalValue::Double(5.0));
        assert_eq!(get("field.size.y").0, GlobalValue::Integer(3));
        // The later file wins.
        assert_eq!(get("bot_id").0, GlobalValue::Integer(1));
        assert!(matches!(get("bot_id").1, GlobalSource::File(f) if f.ends_with("settings.yaml")));
        assert_eq!(
            get("simulation_active"),
            (GlobalValue::Bool(false), GlobalSource::Stack)
        );
        assert_eq!(get("field.name").0, GlobalValue::String("lab".into()));
        assert_eq!(
            get("role"),
            (GlobalValue::String("goalie".into()), GlobalSource::Override)
        );
        assert_eq!(get("ids").0, GlobalValue::IntegerArray(vec![1, 2]));
        assert!(!bb.parameters.contains_key("ignored"));
        let names: Vec<&String> = bb.parameters.keys().collect();
        let mut sorted = names.clone();
        sorted.sort();
        assert_eq!(names, sorted);
    }

    #[test]
    fn field_maps_are_converted_and_missing_ones_skipped() {
        let (stack, dir) = fixture("maps");
        let bb = assemble_with(&stack, &dir, &[]).unwrap();
        assert_eq!(bb.maps.len(), 1, "goals.png does not exist");
        let map = &bb.maps[0];
        assert_eq!(map.topic, "field/map/lines");
        assert_eq!(map.frame_id, "map");
        assert_eq!((map.width, map.height), (3, 2));
        assert_eq!(map.data, [0, 100, 100, 100, 100, 100]);
    }

    #[test]
    fn bright_field_maps_are_rejected() {
        let (stack, dir) = fixture("bright");
        let mut img = image::GrayImage::new(1, 1);
        img.put_pixel(0, 0, image::Luma([255]));
        img.save(dir.join("fields/lab/lines.png")).unwrap();
        let err = format!("{:#}", assemble_with(&stack, &dir, &[]).unwrap_err());
        assert!(err.contains("intensities up to 255"), "{err}");
    }

    #[test]
    fn overrides_must_exist_and_keep_their_type() {
        let (stack, dir) = fixture("overrides");
        let err = |o: (&str, GlobalValue)| {
            format!("{:#}", assemble_with(&stack, &dir, &[o]).unwrap_err())
        };
        assert!(err(("nope", GlobalValue::Integer(1))).contains("unknown global parameter"));
        assert!(err(("bot_id", GlobalValue::String("2".into()))).contains("is a integer"));
        let bb = assemble_with(&stack, &dir, &[("field.size.x", GlobalValue::Integer(6))]).unwrap();
        assert_eq!(bb.parameters["field.size.x"].0, GlobalValue::Double(6.0));
    }

    #[test]
    fn missing_files_are_errors() {
        let (stack, dir) = fixture("missing");
        std::fs::remove_file(dir.join("settings.yaml")).unwrap();
        let err = format!("{:#}", assemble_with(&stack, &dir, &[]).unwrap_err());
        assert!(err.contains("settings.yaml"), "{err}");
    }

    #[test]
    fn parse_override_follows_the_current_type() {
        let int = GlobalValue::Integer(1);
        assert_eq!(
            parse_override("id", "3", &int).unwrap(),
            GlobalValue::Integer(3)
        );
        assert!(parse_override("id", "x", &int).is_err());
        let ip = GlobalValue::String("0.0.0.0".into());
        assert_eq!(
            parse_override("ip", "10", &ip).unwrap(),
            GlobalValue::String("10".into())
        );
        let list = GlobalValue::DoubleArray(vec![1.0]);
        assert_eq!(
            parse_override("l", "[1, 2.5]", &list).unwrap(),
            GlobalValue::DoubleArray(vec![1.0, 2.5])
        );
    }
}
