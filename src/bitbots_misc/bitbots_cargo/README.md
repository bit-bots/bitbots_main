# Rust build dependencies

The workspace root `Cargo.toml` temporarily overrides r2r's binding-generation
crates with a pinned fork commit. The fork updates their bindgen dependency to
handle Clang type definitions correctly, allowing ROS type-support fields to be
generated without a local message-generator workaround.
The dependency update is tracked in the
[upstream r2r pull request](https://github.com/sequenceplanner/r2r/pull/135).

The override covers every r2r crate that directly depends on bindgen so they use
compatible builder and bindings types. See the
[bindgen release notes](https://github.com/rust-lang/rust-bindgen/releases/tag/v0.72.1)
for the upstream Clang compatibility fix.

Once an r2r release includes a compatible bindgen dependency, remove the Git
overrides and update the workspace Cargo lockfile through Pixi. Check the resolved
dependency with `pixi run -e default cargo tree -i bindgen`, then rebuild and test
the Rust packages with freshly generated bindings. r2r caches generated bindings,
so clear its Cargo build artifacts when validating a generator change.
