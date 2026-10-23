//! The vehicle spec's physics on the hero (roadmap 051 K3): sim_vehicle_spec's
//! `carla_physics.yaml`. One writer per mode:
//!
//! - `apply` (acb's own launches, no scenario bridge): acb_bridge puts the file's physics on
//!   the hero when it attaches, so a hero spawned by any client -- demo_scenario.py, CARLA's
//!   own scripts, a hand-made scene -- becomes the spec's car;
//! - `verify` (under a scenario: the CARLA world server applied it after the spawn):
//!   acb_bridge reads the physics back and refuses a car that does not run it.
//!
//! Both refuse a hero whose blueprint is not the file's, naming both.

use carla::{
    client::Vehicle,
    geom::{Location, Vector2D},
    rpc::{GearPhysicsControl, VehiclePhysicsControl},
};
use carla_physics_spec::{CarlaPhysics, ReadBack};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Apply,
    Verify,
}

impl Mode {
    pub fn parse(text: &str) -> Result<Self, String> {
        match text.trim() {
            "apply" => Ok(Self::Apply),
            "verify" => Ok(Self::Verify),
            other => Err(format!("carla_physics_mode {other:?}: apply or verify")),
        }
    }
}

/// `control` with the file's physics: what it says, the rest of the blueprint's kept.
pub fn set(control: &mut VehiclePhysicsControl, p: &CarlaPhysics) {
    let v = &p.vehicle;
    control.mass = v.mass as f32;
    control.center_of_mass = Location {
        x: v.center_of_mass[0] as f32,
        y: v.center_of_mass[1] as f32,
        z: v.center_of_mass[2] as f32,
    };
    control.drag_coefficient = v.drag_coefficient as f32;
    control.max_rpm = v.max_rpm as f32;
    control.torque_curve = v
        .torque_curve
        .iter()
        .map(|[rpm, nm]| Vector2D {
            x: *rpm as f32,
            y: *nm as f32,
        })
        .collect();
    control.final_ratio = v.final_ratio as f32;
    control.forward_gears = v
        .forward_gears
        .iter()
        .map(|g| GearPhysicsControl {
            ratio: g.ratio as f32,
            down_ratio: g.down_ratio as f32,
            up_ratio: g.up_ratio as f32,
        })
        .collect();
    control.use_gear_autobox = v.use_gear_autobox;
    control.moi = v.moi as f32;
    control.damping_rate_full_throttle = v.damping_rate_full_throttle as f32;
    control.damping_rate_zero_throttle_clutch_engaged =
        v.damping_rate_zero_throttle_clutch_engaged as f32;
    control.damping_rate_zero_throttle_clutch_disengaged =
        v.damping_rate_zero_throttle_clutch_disengaged as f32;
    for (wheel, w) in control.wheels.iter_mut().zip(&p.wheels) {
        wheel.tire_friction = w.tire_friction as f32;
        wheel.damping_rate = w.damping_rate as f32;
        wheel.max_steer_angle = w.max_steer_angle_deg as f32;
        wheel.radius = (w.radius_m * 100.0) as f32; // CARLA: cm
        wheel.max_brake_torque = w.max_brake_torque as f32;
        wheel.max_handbrake_torque = w.max_handbrake_torque as f32;
        wheel.lat_stiff_max_load = w.lat_stiff_max_load as f32;
        wheel.lat_stiff_value = w.lat_stiff_value as f32;
        wheel.long_stiff_value = w.long_stiff_value as f32;
    }
}

/// What an actor's physics control says, in the file's units.
pub fn read_back(control: &VehiclePhysicsControl) -> ReadBack {
    ReadBack {
        mass: f64::from(control.mass),
        center_of_mass: [
            f64::from(control.center_of_mass.x),
            f64::from(control.center_of_mass.y),
            f64::from(control.center_of_mass.z),
        ],
        drag_coefficient: f64::from(control.drag_coefficient),
        max_rpm: f64::from(control.max_rpm),
        final_ratio: f64::from(control.final_ratio),
        peak_torque: control
            .torque_curve
            .iter()
            .map(|p| f64::from(p.y))
            .fold(0.0, f64::max),
        moi: f64::from(control.moi),
        damping_rate_zero_throttle_clutch_engaged: f64::from(
            control.damping_rate_zero_throttle_clutch_engaged,
        ),
        wheels: control
            .wheels
            .iter()
            .map(|w| {
                [
                    f64::from(w.tire_friction),
                    f64::from(w.radius) / 100.0,
                    f64::from(w.max_brake_torque),
                    f64::from(w.max_steer_angle),
                    f64::from(w.long_stiff_value),
                ]
            })
            .collect(),
    }
}

/// The road's factor on the tires a world server set (a wet road scales them all alike):
/// the actor's friction over the file's, 1 when they cannot be compared.
fn road_factor(p: &CarlaPhysics, actor: &ReadBack) -> f64 {
    match (p.wheels.first(), actor.wheels.first()) {
        (Some(w), Some(a)) if w.tire_friction > 0.0 => a[0] / w.tire_friction,
        _ => 1.0,
    }
}

/// Why `blueprint` is not the file's car, or None.
pub fn wrong_blueprint(p: &CarlaPhysics, blueprint: &str) -> Option<String> {
    (blueprint != p.blueprint).then(|| {
        format!(
            "the hero is a {blueprint}, but the vehicle spec's car is a {} \
             (carla_physics {}): refusing it",
            p.blueprint,
            &p.spec_hash[..p.spec_hash.len().min(12)]
        )
    })
}

/// Put the spec's physics on the hero (`apply` mode). CARLA applies it at the next tick:
/// [`check`] after one.
pub fn apply(vehicle: &Vehicle, p: &CarlaPhysics) -> Result<(), String> {
    let mut control = vehicle
        .physics_control()
        .map_err(|e| format!("read the hero's physics: {e}"))?;
    set(&mut control, p);
    vehicle
        .apply_physics_control(&control)
        .map_err(|e| format!("apply the vehicle spec's physics: {e}"))
}

/// The hero runs the spec's physics; an error says what it runs instead.
pub fn check(vehicle: &Vehicle, p: &CarlaPhysics, mode: Mode) -> Result<(), String> {
    let back = read_back(
        &vehicle
            .physics_control()
            .map_err(|e| format!("read the hero's physics back: {e}"))?,
    );
    let factor = if mode == Mode::Verify {
        road_factor(p, &back)
    } else {
        1.0
    };
    let off = p.differences(&back, factor);
    if !off.is_empty() {
        return Err(format!(
            "the hero does not run the vehicle spec's physics ({}): {}",
            match mode {
                Mode::Apply => "applied, but the read-back differs",
                Mode::Verify => "the world server should have applied it",
            },
            off.join("; ")
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shipped() -> CarlaPhysics {
        CarlaPhysics::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../deps/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml"
        ))
        .unwrap()
    }

    #[test]
    fn modes_parse() {
        assert_eq!(Mode::parse("apply"), Ok(Mode::Apply));
        assert_eq!(Mode::parse(" verify"), Ok(Mode::Verify));
        assert!(Mode::parse("check").is_err());
    }

    #[test]
    fn another_blueprint_is_refused_with_both_names() {
        let p = shipped();
        assert_eq!(wrong_blueprint(&p, "vehicle.tesla.model3"), None);
        let why = wrong_blueprint(&p, "vehicle.audi.etron").unwrap();
        assert!(why.contains("vehicle.audi.etron") && why.contains("vehicle.tesla.model3"));
    }

    #[test]
    fn a_wet_road_is_still_the_spec_car() {
        let p = shipped();
        let mut back = p.as_read_back();
        for w in &mut back.wheels {
            w[0] *= 0.75;
        }
        assert!((road_factor(&p, &back) - 0.75).abs() < 1e-9);
        assert!(p.differences(&back, road_factor(&p, &back)).is_empty());
    }
}
