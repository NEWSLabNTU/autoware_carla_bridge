//! Tell the crate which CARLA API it is compiled against, as `cfg(carla_0100)`.
//!
//! carla-rust picks its API from the `CARLA_VERSION` environment variable when set, else
//! from the `carla-0xxx` feature on the `carla` dependency in the workspace `Cargo.toml`.
//! Its own `cfg(carla_version_0100)` is private to it and its exported version metadata
//! reaches only direct dependents of carla-sys, so this crate repeats the same precedence.
//! Used where the 0.10 API differs in shape (no wheel `position`) or misbehaves at run time
//! (`wheel_steer_angle` raises on every call on 0.10.0).

use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo::rustc-check-cfg=cfg(carla_0100)");
    println!("cargo::rerun-if-env-changed=CARLA_VERSION");
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let workspace = manifest.join("../../Cargo.toml");
    println!("cargo::rerun-if-changed={}", workspace.display());

    let is_0100 = match env::var("CARLA_VERSION") {
        Ok(v) if !v.is_empty() => v.starts_with("0.10"),
        _ => fs::read_to_string(&workspace)
            .map(|s| {
                s.lines()
                    .filter(|l| !l.trim_start().starts_with('#'))
                    .any(|l| l.contains("\"carla-0100\""))
            })
            .unwrap_or(false),
    };
    if is_0100 {
        println!("cargo::rustc-cfg=carla_0100");
    }
}
