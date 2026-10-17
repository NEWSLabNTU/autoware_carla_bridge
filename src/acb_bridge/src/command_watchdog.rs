//! Brake when Autoware stops commanding the vehicle (docs/issues/024).
//!
//! The bridge applies `/control/command/control_cmd` as it arrives, and CARLA keeps the last
//! `VehicleControl` applied to an actor until another one replaces it. So when the command
//! stream stops -- `vehicle_cmd_gate` crashed, stopped or starved, its container reloading,
//! DDS lost the match -- the ego goes on executing the last command it got, throttle
//! included, for as long as the simulation runs. A drive-by-wire vehicle interface does not
//! do that: it times the command out and stops the vehicle (pacmod_interface's
//! `command_timeout_ms`, 1000 by default). This is that timer.
//!
//! Ages are simulation time, read from the same CARLA frame clock on both sides -- the
//! command is stamped with the frame current when the bridge received it, the check with the
//! frame being processed -- so a paused simulation (no frames) never ages a command, and a
//! bridge loop that stalls on the host does not either: on resuming it drains the queued
//! commands against the new frame before it checks.

/// What the watchdog decided for this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    /// Disabled, or no command has been received yet this vehicle session: leave the
    /// vehicle alone. A bench setup without Autoware must not be held on the brakes.
    Unarmed,
    /// The last command is younger than the timeout; it stands.
    Fresh,
    /// The command just went stale, `age_s` after it arrived: start braking.
    Tripped { age_s: f64 },
    /// Still stale: keep braking.
    Stale { age_s: f64 },
}

impl Verdict {
    /// Whether the bridge must override the last command with a stop.
    pub fn brakes(&self) -> bool {
        matches!(self, Verdict::Tripped { .. } | Verdict::Stale { .. })
    }
}

/// Times out `/control/command/control_cmd`.
#[derive(Debug, Clone)]
pub struct CommandWatchdog {
    timeout_s: f64,
    last_command_s: Option<f64>,
    tripped: bool,
}

impl CommandWatchdog {
    /// `timeout_s <= 0` (or not finite) disables the watchdog.
    pub fn new(timeout_s: f64) -> Self {
        Self {
            timeout_s: if timeout_s.is_finite() && timeout_s > 0.0 {
                timeout_s
            } else {
                0.0
            },
            last_command_s: None,
            tripped: false,
        }
    }

    pub fn enabled(&self) -> bool {
        self.timeout_s > 0.0
    }

    pub fn timeout_s(&self) -> f64 {
        self.timeout_s
    }

    /// A command arrived at simulation time `now_s`. Returns `Some(stale_for_s)` when it ends
    /// a trip -- how long the vehicle was without commands -- so the caller can say so once.
    pub fn on_command(&mut self, now_s: f64) -> Option<f64> {
        let recovered = if self.tripped {
            self.tripped = false;
            Some(self.last_command_s.map_or(0.0, |t| (now_s - t).max(0.0)))
        } else {
            None
        };
        self.last_command_s = Some(now_s);
        recovered
    }

    /// Forget the command history: the next command arms the watchdog afresh. For when the
    /// bridge stops applying Autoware's commands on purpose (MANUAL), so the return to
    /// AUTONOMOUS does not read the gap as a dead stack.
    pub fn disarm(&mut self) {
        self.last_command_s = None;
        self.tripped = false;
    }

    /// The verdict for the frame at simulation time `now_s`.
    pub fn check(&mut self, now_s: f64) -> Verdict {
        if !self.enabled() {
            return Verdict::Unarmed;
        }
        let Some(last) = self.last_command_s else {
            return Verdict::Unarmed;
        };
        // Time going backwards is a new simulation clock (a reloaded world), not a stale
        // command: restart the count from here rather than braking on a negative age.
        if now_s < last {
            self.last_command_s = Some(now_s);
            self.tripped = false;
            return Verdict::Fresh;
        }
        let age_s = now_s - last;
        if age_s <= self.timeout_s {
            return Verdict::Fresh;
        }
        if self.tripped {
            Verdict::Stale { age_s }
        } else {
            self.tripped = true;
            Verdict::Tripped { age_s }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_happens_before_the_first_command() {
        let mut w = CommandWatchdog::new(1.0);
        assert_eq!(w.check(0.0), Verdict::Unarmed);
        assert_eq!(w.check(100.0), Verdict::Unarmed);
    }

    #[test]
    fn a_command_inside_the_timeout_stands() {
        let mut w = CommandWatchdog::new(1.0);
        w.on_command(10.0);
        assert_eq!(w.check(10.0), Verdict::Fresh);
        assert_eq!(w.check(11.0), Verdict::Fresh);
    }

    #[test]
    fn a_silent_stack_trips_once_then_stays_stale() {
        let mut w = CommandWatchdog::new(1.0);
        w.on_command(10.0);
        assert_eq!(
            w.check(11.05),
            Verdict::Tripped {
                age_s: 11.05 - 10.0
            }
        );
        assert!(w.check(11.05).brakes());
        match w.check(15.0) {
            Verdict::Stale { age_s } => assert!((age_s - 5.0).abs() < 1e-9),
            v => panic!("expected Stale, got {v:?}"),
        }
    }

    #[test]
    fn the_next_command_ends_the_trip_and_reports_the_gap() {
        let mut w = CommandWatchdog::new(1.0);
        w.on_command(10.0);
        assert!(w.check(12.0).brakes());
        let gap = w.on_command(14.5).expect("a trip was in progress");
        assert!((gap - 4.5).abs() < 1e-9);
        assert_eq!(w.check(14.5), Verdict::Fresh);
        // and a second trip is reported as a new one
        assert_eq!(w.check(16.0), Verdict::Tripped { age_s: 1.5 });
    }

    #[test]
    fn commands_without_a_trip_report_nothing() {
        let mut w = CommandWatchdog::new(1.0);
        assert_eq!(w.on_command(1.0), None);
        assert_eq!(w.on_command(1.05), None);
    }

    #[test]
    fn a_paused_simulation_does_not_age_the_command() {
        // No frames while paused means no checks, and the clock does not move: the first
        // frame after the pause sees the same age as the last one before it.
        let mut w = CommandWatchdog::new(1.0);
        w.on_command(20.0);
        assert_eq!(w.check(20.05), Verdict::Fresh);
        // ... minutes of wall time with no frame ...
        assert_eq!(w.check(20.10), Verdict::Fresh);
    }

    #[test]
    fn time_going_backwards_restarts_the_count() {
        let mut w = CommandWatchdog::new(1.0);
        w.on_command(500.0);
        assert!(w.check(502.0).brakes());
        assert_eq!(w.check(3.0), Verdict::Fresh);
        assert_eq!(w.check(3.9), Verdict::Fresh);
        assert_eq!(w.check(4.1), Verdict::Tripped { age_s: 4.1 - 3.0 });
    }

    #[test]
    fn disarm_forgets_the_last_command() {
        let mut w = CommandWatchdog::new(1.0);
        w.on_command(10.0);
        w.disarm();
        assert_eq!(w.check(50.0), Verdict::Unarmed);
        w.on_command(50.0);
        assert_eq!(w.check(50.5), Verdict::Fresh);
    }

    #[test]
    fn zero_negative_or_nan_timeout_disables() {
        for t in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut w = CommandWatchdog::new(t);
            assert!(!w.enabled(), "{t}");
            w.on_command(0.0);
            assert_eq!(w.check(1e6), Verdict::Unarmed, "{t}");
        }
    }
}
