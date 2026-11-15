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
//!
//! And both keep at it while the hero drives ([`Keeper`]): CARLA 0.9.16 puts a blueprint's
//! own tire friction (3.5 for the Model 3) back on live vehicles of that blueprint whenever
//! another vehicle of it spawns, by any client (measured 2026-11-15: the car whose physics
//! was applied last loses it; the rest of the physics stays, and a sensor, a walker or
//! another blueprint resets nothing). After
//! each spawn the frames show, the hero is read back for up to [`GRACE_FRAMES`] frames:
//! `apply` puts the file back on every frame it reads it off (measured: back on the next
//! frame); `verify` waits for the world server, which puts it back a tick after it sees the
//! spawn. Either says so loudly if it stays off.

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
/// the actor's friction over the file's, 1 when they cannot be compared. A road only takes
/// grip away (the world server's factor is the scenario's scale, at most 1 in every
/// scenario here, times the wetness's): above 1 it is not a road but the blueprint's own
/// tires -- CARLA's reset after a spawn -- so it is not divided out, and the check names the
/// friction.
fn road_factor(p: &CarlaPhysics, actor: &ReadBack) -> f64 {
    match (p.wheels.first(), actor.wheels.first()) {
        (Some(w), Some(a)) if w.tire_friction > 0.0 => {
            let f = a[0] / w.tire_friction;
            if f > 1.0 + 1e-3 {
                1.0
            } else {
                f
            }
        }
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

/// Frames the hero has, after a spawn, to be on the file again before it is called off:
/// `verify` waits for the world server, which re-applies after the tick that showed it the
/// spawn; `apply` re-applies each frame it reads the hero off (CARLA answers a read sent
/// right after an apply with the old control: the apply is asynchronous).
pub const GRACE_FRAMES: u32 = 20;

/// What the hero runs after a frame, as [`Keeper::frame`] found it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Kept {
    /// Nothing spawned, or nothing to look at this frame.
    Quiet,
    /// The hero runs the file.
    Runs,
    /// It had lost the file and has it back (what was off).
    Reapplied(String),
    /// Off, and the grace is not over yet (`apply` re-applied it this frame).
    Waiting,
    /// Off past the grace: why.
    Off(String),
}

/// Keeps the hero on the file after attach: a spawn (a new highest actor id in a frame)
/// starts a look of up to [`GRACE_FRAMES`] frames.
#[derive(Debug)]
pub struct Keeper {
    mode: Mode,
    highest: Option<u32>,
    /// Frames left to see the file back on.
    grace: Option<u32>,
    /// What was off, once it was, so its return is told.
    was_off: Option<String>,
}

impl Keeper {
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            highest: None,
            grace: None,
            was_off: None,
        }
    }

    /// One frame whose highest actor id is `highest`. `check` reads the hero back (an error
    /// says what is off); `apply` puts the file on it (`apply` mode only).
    pub fn frame(
        &mut self,
        highest: u32,
        check: impl FnOnce() -> Result<(), String>,
        apply: impl FnOnce() -> Result<(), String>,
    ) -> Kept {
        // The attach checked the hero already: the first frame only starts the watch.
        if self.highest.is_some_and(|h| h != highest) {
            self.grace = Some(GRACE_FRAMES);
        }
        self.highest = Some(highest);
        let Some(left) = self.grace else {
            return Kept::Quiet;
        };
        match check() {
            Ok(()) => {
                self.grace = None;
                match self.was_off.take() {
                    Some(off) => Kept::Reapplied(off),
                    None => Kept::Runs,
                }
            }
            Err(off) if left > 1 => {
                self.grace = Some(left - 1);
                let off = match self.mode {
                    Mode::Apply => match apply() {
                        Ok(()) => off,
                        Err(e) => format!("{off}; re-applying failed: {e}"),
                    },
                    Mode::Verify => off,
                };
                // The first thing found off is what was lost; later reads add nothing.
                self.was_off.get_or_insert(off);
                Kept::Waiting
            }
            Err(off) => {
                self.grace = None;
                self.was_off = None;
                Kept::Off(off)
            }
        }
    }
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

    #[test]
    fn the_blueprints_own_tires_are_not_a_road() {
        let p = shipped();
        let mut back = p.as_read_back();
        for w in &mut back.wheels {
            w[0] = 3.5; // CARLA's reset after another Model 3 spawned
        }
        assert_eq!(road_factor(&p, &back), 1.0);
        let off = p.differences(&back, road_factor(&p, &back));
        assert!(off.iter().any(|d| d.contains("tire_friction")), "{off:?}");
    }

    #[test]
    fn apply_puts_the_file_back_after_a_spawn_only() {
        let mut k = Keeper::new(Mode::Apply);
        let never = || -> Result<(), String> { panic!("no look without a spawn") };
        assert_eq!(
            k.frame(40, never, || Ok(())),
            Kept::Quiet,
            "the attach looked"
        );
        assert_eq!(k.frame(40, never, || Ok(())), Kept::Quiet);
        // A sensor of another client: looked at, fine.
        assert_eq!(
            k.frame(41, || Ok(()), || panic!("nothing to apply")),
            Kept::Runs
        );
        assert_eq!(k.frame(41, never, || panic!()), Kept::Quiet);
        // Another Model 3: off, applied; the next frame reads it back.
        let mut applied = false;
        let kept = k.frame(
            42,
            || Err("tire_friction".into()),
            || {
                applied = true;
                Ok(())
            },
        );
        assert!(applied);
        assert_eq!(kept, Kept::Waiting);
        assert_eq!(
            k.frame(42, || Ok(()), || panic!()),
            Kept::Reapplied("tire_friction".into())
        );
        assert_eq!(k.frame(42, never, || panic!()), Kept::Quiet);
    }

    #[test]
    fn verify_waits_for_the_world_server_then_calls_it_off() {
        let mut k = Keeper::new(Mode::Verify);
        k.frame(40, || panic!("the attach looked"), || panic!());
        assert_eq!(k.frame(41, || Err("f".into()), || panic!()), Kept::Waiting);
        assert_eq!(
            k.frame(41, || Ok(()), || panic!()),
            Kept::Reapplied("f".into())
        );
        assert_eq!(
            k.frame(41, || panic!("done looking"), || panic!()),
            Kept::Quiet
        );
        assert_eq!(
            k.frame(42, || Err("f".into()), || panic!("verify never applies")),
            Kept::Waiting
        );
        for _ in 0..GRACE_FRAMES - 2 {
            assert_eq!(k.frame(42, || Err("f".into()), || panic!()), Kept::Waiting);
        }
        assert_eq!(
            k.frame(42, || Err("f".into()), || panic!()),
            Kept::Off("f".into())
        );
        assert_eq!(k.frame(42, || panic!("done"), || panic!()), Kept::Quiet);
    }
}
