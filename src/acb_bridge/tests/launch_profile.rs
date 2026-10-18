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

/// XML forbids `--` inside a comment, and play_launch's parser refuses the whole launch for
/// it (minutes in, after `just ego-av` has started).
#[test]
fn launch_comments_have_no_double_hyphen() {
    for rel in [
        "launch/carla_simulator.launch.xml",
        "launch/components/tier4_control_component.launch.xml",
        "launch/components/tier4_perception_component.launch.xml",
        "launch/components/perception.launch.xml",
    ] {
        let text = read(rel);
        for comment in text.split("<!--").skip(1) {
            let body = comment.split("-->").next().unwrap_or("");
            assert!(!body.contains("--"), "{rel}: comment contains '--': {body}");
        }
    }
}
