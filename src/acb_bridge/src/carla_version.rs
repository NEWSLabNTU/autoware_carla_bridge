//! Refuse a CARLA server whose version this binary was not built for.
//!
//! carla-rust compiles against one CARLA client API, chosen at build time (the
//! `carla-0916` feature in the workspace `Cargo.toml`, or `CARLA_VERSION`). A client built
//! for 0.9.16 talking to a 0.9.15 server does not fail cleanly: it fails somewhere in RPC
//! with a msgpack error. Comparing versions on connect turns that into one clear message.

/// The CARLA version this binary was built for, e.g. `"0.9.16"`.
pub const BUILT_FOR: &str = carla::CARLA_VERSION;

/// `"0.9.16"`, `"0.9.16-dirty"`, `"0.10.0"`, `"1.0"`, `"1.0-82c52a7c2"` → `(major, minor, patch)`.
///
/// Only the leading numeric components count: CARLA reports suffixes such as `-dirty` or
/// `-12-gabc1234` for builds from a git tree, and from 1.0 a source or nightly build reports
/// `<release>-<commit>`. 1.0 names its release with two components (`"1.0"`); a missing
/// patch is 0. The same rules as scensim_carla_world's `version.rs` (roadmap 083).
pub fn parse_version(text: &str) -> Option<(u32, u32, u32)> {
    let text = text.trim().trim_start_matches('v');
    let release = text.split('-').next()?;
    let mut parts = release.split('.');
    let major = parts.next()?.parse().ok()?;
    let minor = parts.next()?.parse().ok()?;
    let patch = match parts.next() {
        Some(p) => p.parse().ok()?,
        None => 0,
    };
    if parts.next().is_some() {
        return None;
    }
    Some((major, minor, patch))
}

/// Whether a server reporting `server` can serve a client built for `built_for`.
///
/// CARLA's RPC changes between releases (0.9.15 → 0.9.16 → 0.10.0 → 1.0), so the release --
/// major.minor.patch -- must match; build suffixes are ignored.
/// `Err` carries a message naming both versions.
pub fn check(built_for: &str, server: &str) -> Result<(), String> {
    let built = parse_version(built_for);
    let served = parse_version(server);
    match (built, served) {
        (Some(b), Some(s)) if b == s => Ok(()),
        (Some(_), Some(_)) => Err(format!(
            "CARLA version mismatch: the server is {server} but this binary was built for \
             CARLA {built_for}. Run a CARLA {built_for} server, or rebuild with the matching \
             carla-rust version feature (or CARLA_VERSION=<server version>)"
        )),
        _ => Err(format!(
            "cannot compare CARLA versions: server reports '{server}', binary built for \
             '{built_for}'"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_release_and_suffixed_versions() {
        assert_eq!(parse_version("0.9.16"), Some((0, 9, 16)));
        assert_eq!(parse_version("0.9.16-dirty"), Some((0, 9, 16)));
        assert_eq!(parse_version("0.9.16-12-gabc1234"), Some((0, 9, 16)));
        assert_eq!(parse_version("0.10.0"), Some((0, 10, 0)));
        assert_eq!(parse_version("1.0"), Some((1, 0, 0)));
        assert_eq!(parse_version("1.0-82c52a7c2"), Some((1, 0, 0)));
        assert_eq!(parse_version("1.0.0.1"), None);
        assert_eq!(parse_version("abc1234"), None);
        assert_eq!(parse_version(""), None);
    }

    #[test]
    fn same_release_passes() {
        assert!(check("0.9.16", "0.9.16").is_ok());
        assert!(check("0.9.16", "0.9.16-dirty").is_ok());
        assert!(check("0.10.0", "0.10.0").is_ok());
        assert!(check("1.0", "1.0").is_ok());
        assert!(check("1.0", "1.0-82c52a7c2").is_ok());
    }

    #[test]
    fn other_release_is_refused_with_both_versions() {
        let err = check("0.9.16", "0.9.15").unwrap_err();
        assert!(err.contains("0.9.16") && err.contains("0.9.15"), "{err}");
        let err = check("0.9.16", "0.10.0").unwrap_err();
        assert!(err.contains("0.9.16") && err.contains("0.10.0"), "{err}");
        let err = check("0.9.16", "1.0").unwrap_err();
        assert!(err.contains("0.9.16") && err.contains("1.0"), "{err}");
        let err = check("1.0", "0.9.16").unwrap_err();
        assert!(err.contains("1.0") && err.contains("0.9.16"), "{err}");
    }

    #[test]
    fn unparsable_server_version_is_refused() {
        let err = check("0.9.16", "abc1234").unwrap_err();
        assert!(err.contains("abc1234"), "{err}");
    }

    #[test]
    fn built_for_is_a_release() {
        assert!(parse_version(BUILT_FOR).is_some(), "{BUILT_FOR}");
    }
}
