//! The CARLA launch profile in acb_launch: settings that are not code but that a run depends
//! on, checked here so a sync with upstream or an edit cannot drop them silently.

use std::path::PathBuf;

fn acb_launch() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../acb_launch")
}

fn read(rel: &str) -> String {
    let path = acb_launch().join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

fn yaml(text: &str) -> serde_yaml::Value {
    serde_yaml::from_str(text).expect("valid YAML")
}

fn mpc_params(doc: &serde_yaml::Value) -> &serde_yaml::Value {
    &doc["/**"]["ros__parameters"]
}

const MPC: &str = "config/control/trajectory_follower/lateral/mpc.param.yaml";
const UPSTREAM_MPC: &str =
    "/opt/autoware/1.5.0/share/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml";

/// Issue 025: MPC's steering offset estimator learns from every frame above 5.56 m/s with
/// a small steering angle, collisions and transients included, never forgets across
/// episodes, and adds up to 0.02 rad to every command. CARLA's steering has no offset to
/// learn, so the CARLA profile turns it off.
#[test]
fn mpc_steering_offset_removal_is_off() {
    let doc = yaml(&read(MPC));
    let offset = &mpc_params(&doc)["steering_offset"];
    assert_eq!(
        offset["enable_auto_steering_offset_removal"].as_bool(),
        Some(false),
        "{MPC}: steering_offset.enable_auto_steering_offset_removal must be false (issue 025)"
    );
}

/// The copy differs from upstream's in that one value only (scripts/sync_control_component.sh).
#[test]
fn mpc_copy_matches_upstream_but_for_the_offset() {
    let Ok(upstream) = std::fs::read_to_string(UPSTREAM_MPC) else {
        eprintln!("no upstream at {UPSTREAM_MPC}; skipped");
        return;
    };
    let mut up = yaml(&upstream);
    let mut ours = yaml(&read(MPC));
    for doc in [&mut up, &mut ours] {
        doc["/**"]["ros__parameters"]["steering_offset"]["enable_auto_steering_offset_removal"] =
            serde_yaml::Value::Null;
    }
    assert_eq!(
        up, ours,
        "{MPC} drifted from upstream; run scripts/sync_control_component.sh"
    );
}

/// carla_simulator.launch.xml launches control from acb's component copy, with acb's MPC
/// parameters, and keeps autoware.launch.xml from launching a second controller.
#[test]
fn carla_profile_launches_control_with_acb_mpc() {
    let launch = read("launch/carla_simulator.launch.xml");
    assert!(
        launch.contains(
            "<arg name=\"launch_control\" value=\"$(eval &quot;'$(var control)' == 'true' and '$(var carla_control)' != 'true'&quot;)\"/>"
        ),
        "autoware.launch.xml must not launch control when carla_control is on"
    );
    assert!(
        launch.contains(
            "$(find-pkg-share acb_launch)/launch/components/tier4_control_component.launch.xml"
        ),
        "acb's control component is not included"
    );
    assert!(
        launch.contains(&format!("$(find-pkg-share acb_launch)/{MPC}")),
        "acb's MPC parameters are not passed"
    );
    let component = read("launch/components/tier4_control_component.launch.xml");
    assert!(component.contains("<arg name=\"lat_controller_param_path\" default="));
    assert!(component.contains("value=\"$(var lat_controller_param_path)\""));
}

/// Issue 026: `perception:=false` (ground-truth objects) must still launch what planning and
/// the diagnostic graph need from the perception component -- obstacle segmentation, the
/// occupancy grid, traffic light fusion -- and drop only object recognition.
#[test]
fn perception_off_keeps_everything_but_object_recognition() {
    let launch = read("launch/carla_simulator.launch.xml");
    assert!(
        launch.contains("<group if=\"$(eval &quot;'$(var carla_perception)' == 'true'&quot;)\">"),
        "acb's perception component must not be gated on `perception`"
    );
    assert!(
        launch.contains(
            "<arg name=\"object_recognition\" value=\"$(eval &quot;'$(var perception)' == 'true' and '$(var publish_ground_truth_objects)' != 'true'&quot;)\"/>"
        ),
        "object recognition must follow `perception` and give way to ground truth"
    );
    let component = read("launch/components/tier4_perception_component.launch.xml");
    assert!(
        component.contains("$(find-pkg-share acb_launch)/launch/components/perception.launch.xml")
    );
    assert!(component
        .contains("<arg name=\"object_recognition\" value=\"$(var object_recognition)\"/>"));
    let perception = read("launch/components/perception.launch.xml");
    assert!(perception.contains("<arg name=\"object_recognition\" default=\"true\""));
    assert_eq!(
        perception.matches("'$(var object_recognition)'.lower() == 'true' and '$(var use_empty_dynamic_object_publisher)'.lower()").count(),
        2,
        "both object recognition groups (pipeline and empty publisher) must be gated, case-blind: \
         the value carla_simulator passes is an $(eval), which yields True/False"
    );
    assert!(!perception.contains("<group unless=\"$(var use_empty_dynamic_object_publisher)\">"));
}

const TRACKER: &str =
    "config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml";
const UPSTREAM_TRACKER: &str = "/opt/autoware/1.5.0/share/autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml";

/// Issue 027: two UNKNOWN trackers within 9 m are merged when their generalized IoU exceeds
/// UNKNOWN's pruning threshold. Upstream's -0.3 in Autoware 1.5.0 admits pairs that do not
/// overlap at all, so a pedestrian standing 1.4 m from a parked car was merged into it,
/// frame after frame. Upstream itself later moved to 0.1 (autoware_launch #1885, #1907).
#[test]
fn tracker_merges_only_overlapping_unknowns() {
    let doc = yaml(&read(TRACKER));
    let giou = &doc["/**"]["ros__parameters"]["pruning_generalized_iou_thresholds"];
    assert_eq!(
        giou[0].as_f64(),
        Some(0.1),
        "{TRACKER}: pruning_generalized_iou_thresholds[UNKNOWN] must be 0.1 (issue 027)"
    );
}

/// The copy differs from upstream's in UNKNOWN's GIoU threshold only
/// (scripts/sync_perception_component.sh).
#[test]
fn tracker_copy_matches_upstream_but_for_unknown_giou() {
    let Ok(upstream) = std::fs::read_to_string(UPSTREAM_TRACKER) else {
        eprintln!("no upstream at {UPSTREAM_TRACKER}; skipped");
        return;
    };
    let mut up = yaml(&upstream);
    let mut ours = yaml(&read(TRACKER));
    for doc in [&mut up, &mut ours] {
        doc["/**"]["ros__parameters"]["pruning_generalized_iou_thresholds"][0] =
            serde_yaml::Value::Null;
    }
    assert_eq!(
        up, ours,
        "{TRACKER} drifted from upstream; run scripts/sync_perception_component.sh"
    );
}

/// acb's perception component hands the tracker acb's parameter file.
#[test]
fn perception_component_passes_acb_tracker_params() {
    let component = read("launch/components/tier4_perception_component.launch.xml");
    assert!(
        component.contains(&format!("value=\"$(find-pkg-share acb_launch)/{TRACKER}\"")),
        "acb's multi_object_tracker parameters are not passed"
    );
    assert!(!component.contains(&format!("$(find-pkg-share autoware_launch)/{TRACKER}")));
}

const UPSTREAM_COMMON: &str = "/opt/autoware/1.5.0/share/autoware_launch/config/planning/scenario_planning/common/common.param.yaml";

/// The value of `<arg name="{name}" default="..."/>` in a launch file.
fn arg_default<'a>(launch: &'a str, name: &str) -> &'a str {
    let at = launch
        .find(&format!("<arg name=\"{name}\" default=\""))
        .unwrap_or_else(|| panic!("no arg {name} with a default"));
    let rest = &launch[at..];
    let start = rest.find("default=\"").unwrap() + "default=\"".len();
    let len = rest[start..].find('"').unwrap();
    &rest[start..start + len]
}

/// Roadmap 033: `max_velocity` sets Autoware's max_vel. Its default is upstream's own value,
/// so every stack that does not pass it plans exactly as upstream does (15 km/h).
#[test]
fn max_velocity_defaults_to_upstream_max_vel() {
    let launch = read("launch/carla_simulator.launch.xml");
    let default: f64 = arg_default(&launch, "max_velocity").parse().unwrap();
    assert_eq!(default, 4.17);
    let Ok(upstream) = std::fs::read_to_string(UPSTREAM_COMMON) else {
        eprintln!("no upstream at {UPSTREAM_COMMON}; skipped");
        return;
    };
    let up = yaml(&upstream);
    assert_eq!(
        up["/**"]["ros__parameters"]["max_vel"].as_f64(),
        Some(default),
        "max_velocity's default must stay upstream's max_vel"
    );
}

/// carla_simulator.launch.xml launches planning from acb's component copy with the common
/// parameter file planning_common_param picks for max_velocity, keeps autoware.launch.xml
/// from launching a second planner, and starts the trajectory relay autoware.launch.xml
/// starts beside planning.
#[test]
fn carla_profile_launches_planning_with_max_velocity() {
    let launch = read("launch/carla_simulator.launch.xml");
    assert!(
        launch.contains(
            "<arg name=\"launch_planning\" value=\"$(eval &quot;'$(var planning)' == 'true' and '$(var carla_planning)' != 'true'&quot;)\"/>"
        ),
        "autoware.launch.xml must not launch planning when carla_planning is on"
    );
    assert!(launch.contains(
        "<group if=\"$(eval &quot;'$(var planning)' == 'true' and '$(var carla_planning)' == 'true'&quot;)\">"
    ));
    assert!(launch.contains(
        "$(find-pkg-share acb_launch)/launch/components/tier4_planning_component.launch.xml"
    ));
    assert!(
        launch.contains(
            "value=\"$(command '$(find-pkg-prefix acb_launch)/lib/acb_launch/planning_common_param $(find-pkg-share autoware_launch)/config/planning/scenario_planning/common/common.param.yaml $(var max_velocity)')\""
        ),
        "the planning common parameter file must come from planning_common_param and max_velocity"
    );
    assert!(launch.contains("<node pkg=\"topic_tools\" exec=\"relay\" name=\"trajectory_relay\">"));
    let component = read("launch/components/tier4_planning_component.launch.xml");
    assert!(component.contains("<arg name=\"planning_common_param_path\" default="));
    assert!(component
        .contains("<arg name=\"common_param_path\" value=\"$(var planning_common_param_path)\"/>"));
}

/// Runs acb_launch's planning_common_param on `upstream` with `max_velocity`, its cache in
/// `cache`; returns (exit code, stdout, stderr).
fn planning_common_param(
    upstream: &str,
    max_velocity: &str,
    cache: &std::path::Path,
) -> (i32, String, String) {
    let out = std::process::Command::new("python3")
        .arg(acb_launch().join("scripts/planning_common_param"))
        .args([upstream, max_velocity])
        .env("XDG_CACHE_HOME", cache)
        .output()
        .expect("python3 runs");
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8(out.stdout).unwrap(),
        String::from_utf8(out.stderr).unwrap(),
    )
}

/// At upstream's value the script hands back upstream's path (the default stack loads
/// upstream's file); at another it writes upstream's file with max_vel changed and nothing
/// else; a bad value fails the launch. Stdout is the bare path and stderr stays empty, as a
/// strict $(command) substitution needs.
#[test]
fn planning_common_param_changes_max_vel_only() {
    let Ok(upstream) = std::fs::read_to_string(UPSTREAM_COMMON) else {
        eprintln!("no upstream at {UPSTREAM_COMMON}; skipped");
        return;
    };
    let cache =
        std::env::temp_dir().join(format!("acb_planning_common_param_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&cache);

    let (code, out, err) = planning_common_param(UPSTREAM_COMMON, "4.17", &cache);
    assert_eq!((code, out.as_str(), err.as_str()), (0, UPSTREAM_COMMON, ""));
    assert!(!cache.exists(), "the default must not write a file");

    for (arg, want) in [("11.11", 11.11), ("20", 20.0)] {
        let (code, out, err) = planning_common_param(UPSTREAM_COMMON, arg, &cache);
        assert_eq!((code, err.as_str()), (0, ""), "{arg}");
        assert!(
            !out.ends_with('\n') && out.starts_with(cache.to_str().unwrap()),
            "{out}"
        );
        let mut ours = yaml(&std::fs::read_to_string(&out).unwrap());
        // A float, also for a whole number: the planners declare max_vel a double.
        assert_eq!(
            ours["/**"]["ros__parameters"]["max_vel"].as_f64(),
            Some(want),
            "{arg}"
        );
        let mut up = yaml(&upstream);
        for doc in [&mut up, &mut ours] {
            doc["/**"]["ros__parameters"]["max_vel"] = serde_yaml::Value::Null;
        }
        assert_eq!(up, ours, "only max_vel may differ from upstream ({arg})");
    }

    for bad in ["abc", "0", "-1", "nan", "inf"] {
        let (code, out, _) = planning_common_param(UPSTREAM_COMMON, bad, &cache);
        assert_eq!((code, out.as_str()), (2, ""), "{bad}");
    }
    let _ = std::fs::remove_dir_all(&cache);
}

/// XML forbids `--` inside a comment, and play_launch's parser refuses the whole launch for
/// it (minutes in, after `just ego-av` has started).
#[test]
fn launch_comments_have_no_double_hyphen() {
    for rel in [
        "launch/carla_simulator.launch.xml",
        "launch/components/tier4_control_component.launch.xml",
        "launch/components/tier4_perception_component.launch.xml",
        "launch/components/tier4_planning_component.launch.xml",
        "launch/components/perception.launch.xml",
    ] {
        let text = read(rel);
        for comment in text.split("<!--").skip(1) {
            let body = comment.split("-->").next().unwrap_or("");
            assert!(!body.contains("--"), "{rel}: comment contains '--': {body}");
        }
    }
}
