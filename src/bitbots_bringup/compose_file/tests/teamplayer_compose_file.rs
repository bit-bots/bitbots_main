//! Keeps the shipped compose file consistent with the workspace it refers to.

use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use bitbots_compose_file::{ProcessSpec, Stack};

fn bringup_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn stack() -> Stack {
    Stack::load(&bringup_dir().join("config/teamplayer.yaml")).unwrap()
}

/// Packages of the source workspace, by name.
fn workspace_packages() -> HashMap<String, PathBuf> {
    fn walk(dir: &Path, found: &mut HashMap<String, PathBuf>) {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        let manifest = dir.join("package.xml");
        if let Ok(xml) = std::fs::read_to_string(&manifest) {
            if let Some(name) = xml
                .split_once("<name>")
                .and_then(|(_, rest)| rest.split_once("</name>"))
                .map(|(name, _)| name.trim().to_string())
            {
                found.insert(name, dir.to_path_buf());
            }
        }
        for entry in entries.flatten() {
            let path = entry.path();
            let hidden = entry.file_name().to_string_lossy().starts_with('.');
            if path.is_dir() && !hidden {
                walk(&path, found);
            }
        }
    }
    let mut found = HashMap::new();
    walk(&bringup_dir().join(".."), &mut found);
    found
}

fn file_exists_below(dir: &Path, file: &str) -> bool {
    std::fs::read_dir(dir).is_ok_and(|entries| {
        entries.flatten().any(|e| {
            let path = e.path();
            if path.is_dir() {
                file_exists_below(&path, file)
            } else {
                path.file_name().is_some_and(|n| n == file)
            }
        })
    })
}

#[test]
fn every_profile_resolves_without_problems() {
    let stack = stack();
    let defaults = stack.resolve(&stack.default_values(), &stack.default_enabled());
    assert!(defaults.problems.is_empty(), "{:?}", defaults.problems);
    for profile in stack.profiles() {
        let mut values = stack.default_values();
        let mut enabled = stack.default_enabled();
        stack
            .apply_profile(&profile.name, &mut values, &mut enabled)
            .unwrap();
        let resolved = stack.resolve(&values, &enabled);
        assert!(
            resolved.problems.is_empty(),
            "profile {}: {:?}",
            profile.name,
            resolved.problems
        );
    }
}

#[test]
fn referenced_launch_files_exist() {
    let packages = workspace_packages();
    for c in stack().components() {
        for p in &c.processes {
            if let ProcessSpec::Launch(l) = p {
                let Some(dir) = packages.get(&l.package) else {
                    panic!(
                        "{}: launch package {} is not in the workspace",
                        c.key, l.package
                    );
                };
                assert!(
                    file_exists_below(dir, &l.file),
                    "{}: {} not found in {}",
                    c.key,
                    l.file,
                    dir.display()
                );
            }
        }
    }
}

#[test]
fn referenced_packages_are_declared_in_package_xml() {
    let manifest = std::fs::read_to_string(bringup_dir().join("package.xml")).unwrap();
    let declared =
        |pkg: &str| pkg == "bitbots_bringup" || manifest.contains(&format!(">{pkg}</exec_depend>"));
    for c in stack().components() {
        for p in &c.processes {
            let pkg = match p {
                ProcessSpec::Launch(l) => &l.package,
                ProcessSpec::Run(r) => &r.package,
                ProcessSpec::Command(_) => continue,
            };
            assert!(
                declared(pkg),
                "{}: {pkg} is missing as exec_depend in bitbots_bringup/package.xml",
                c.key
            );
        }
    }
}
