//! What CARLA's vehicle model lacks between a pedal and the wheel (roadmap 051 K3): the
//! brake hydraulics' dead time and pressure build-up, and the motor's torque lag. The spec
//! (sim_vehicle_spec, `carla_physics.yaml`'s `command_lags`) gives them; Isaac's `ev`
//! powertrain models the same numbers in its physics. Applied to the pedals acb_bridge sends,
//! on simulation time, each command advancing the filter to its own time.

use std::collections::VecDeque;

/// The lags, seconds; all zero is no filter.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Lags {
    pub brake_dead_time: f64,
    pub brake_time_constant: f64,
    pub motor_time_constant: f64,
}

impl Lags {
    pub fn is_off(&self) -> bool {
        self.brake_dead_time <= 0.0
            && self.brake_time_constant <= 0.0
            && self.motor_time_constant <= 0.0
    }
}

/// The filter's state across commands.
#[derive(Debug, Default)]
pub struct PedalLag {
    lags: Lags,
    /// Brake requests not yet past the dead time: (time, request).
    pending: VecDeque<(f64, f32)>,
    /// The request the dead time has let through.
    brake_request: f32,
    brake: f32,
    throttle: f32,
    last: Option<f64>,
}

/// One first-order step towards `target` over `dt` with time constant `tau`.
fn first_order(value: f32, target: f32, dt: f64, tau: f64) -> f32 {
    if tau <= 0.0 {
        return target;
    }
    let k = 1.0 - (-dt / tau).exp();
    value + (target - value) * k as f32
}

impl PedalLag {
    pub fn new(lags: Lags) -> Self {
        Self {
            lags,
            ..Default::default()
        }
    }

    /// The pedals the car gets at `now` (s, simulation time) for the request
    /// (`throttle`, `brake`). Time going backwards (a new episode) starts over.
    pub fn step(&mut self, throttle: f32, brake: f32, now: f64) -> (f32, f32) {
        if self.lags.is_off() {
            return (throttle, brake);
        }
        let dt = match self.last {
            Some(last) if now >= last => now - last,
            _ => {
                // First command, or the clock went back: no history to lag behind.
                self.pending.clear();
                self.brake_request = brake;
                self.brake = brake;
                self.throttle = throttle;
                self.last = Some(now);
                return (throttle, brake);
            }
        };
        self.last = Some(now);
        self.pending.push_back((now, brake));
        while let Some(&(t, b)) = self.pending.front() {
            if now - t + 1e-9 >= self.lags.brake_dead_time {
                self.brake_request = b;
                self.pending.pop_front();
            } else {
                break;
            }
        }
        self.brake = first_order(
            self.brake,
            self.brake_request,
            dt,
            self.lags.brake_time_constant,
        );
        self.throttle = first_order(self.throttle, throttle, dt, self.lags.motor_time_constant);
        (self.throttle.clamp(0.0, 1.0), self.brake.clamp(0.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPEC: Lags = Lags {
        brake_dead_time: 0.08,
        brake_time_constant: 0.06,
        motor_time_constant: 0.04,
    };

    #[test]
    fn no_lags_pass_the_pedals_through() {
        let mut f = PedalLag::new(Lags::default());
        assert_eq!(f.step(0.3, 0.0, 0.0), (0.3, 0.0));
        assert_eq!(f.step(0.0, 0.7, 0.05), (0.0, 0.7));
    }

    #[test]
    fn the_brake_waits_the_dead_time_then_builds_up() {
        let mut f = PedalLag::new(SPEC);
        f.step(0.0, 0.0, 0.0);
        // A full brake at 0.1 s: nothing until 0.18 s.
        let (_, b) = f.step(0.0, 1.0, 0.1);
        assert_eq!(b, 0.0);
        let (_, b) = f.step(0.0, 1.0, 0.15);
        assert_eq!(b, 0.0);
        // Past the dead time by 0.2 s: the build-up runs from the previous command (0.15 s),
        // so it starts within one command interval of the dead time's end.
        let (_, b) = f.step(0.0, 1.0, 0.2);
        assert!(b > 0.2 && b < 0.6, "{b}");
        let mut b = b;
        for i in 0..10 {
            b = f.step(0.0, 1.0, 0.25 + 0.05 * f64::from(i)).1;
        }
        assert!(b > 0.99, "{b}");
    }

    #[test]
    fn the_motor_follows_with_its_time_constant() {
        let mut f = PedalLag::new(SPEC);
        f.step(0.0, 0.0, 0.0);
        let (t, _) = f.step(1.0, 0.0, 0.04);
        assert!((t - (1.0 - (-1.0f32).exp())).abs() < 1e-4, "{t}");
    }

    #[test]
    fn a_clock_going_back_starts_over() {
        let mut f = PedalLag::new(SPEC);
        f.step(0.0, 0.0, 10.0);
        f.step(0.0, 1.0, 10.05);
        assert_eq!(f.step(0.2, 0.0, 0.0), (0.2, 0.0));
    }
}
