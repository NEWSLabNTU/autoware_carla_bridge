//! Turn a requested longitudinal acceleration into a CARLA pedal, by measurement.
//!
//! The bridge used to divide by a single constant:
//!
//! ```text
//! control.throttle = (accel / 3.0).clamp(0.0, 1.0)
//! control.brake    = (-accel / 3.0).clamp(0.0, 1.0)
//! ```
//!
//! which assumes the response is linear in the pedal, the same at every speed, and the same
//! for the engine and the brakes. Measured on CARLA 0.9.16's Tesla Model 3, none of those
//! hold. Full throttle delivers about 6 m/s^2, not 3. Half throttle delivers 2.6 m/s^2 at
//! rest and 0.12 m/s^2 at 14 m/s -- the same pedal, a factor of twenty apart. Full brake
//! ranges from 5.8 to 11.2 m/s^2 depending on speed. And the car decelerates at 2.6 to 8.8
//! m/s^2 with *no* pedal at all, so a request for gentle braking may need no brake.
//!
//! So the conversion is a lookup into a measured table instead, in the same shape Autoware's
//! own `raw_vehicle_cmd_converter` uses: rows are pedal positions, columns are speeds, cells
//! are the acceleration that combination produces.
//!
//! ```text
//! default,0.0,2.0,4.0,...
//! 0.20,0.060,0.060,0.010,...
//! ```
//!
//! Regenerate the tables for a different vehicle with `scripts/probe_longitudinal.py`, which
//! documents how they were measured and what earlier measurement methods got wrong.

use std::{fs, path::Path};

use crate::error::{BridgeError, Result};

/// One measured table: acceleration as a function of pedal position and speed.
#[derive(Debug, Clone)]
pub struct PedalMap {
    /// Column headings, ascending.
    speeds: Vec<f64>,
    /// Row headings, ascending.
    pedals: Vec<f64>,
    /// `accel[pedal_index][speed_index]`.
    accel: Vec<Vec<f64>>,
}

impl PedalMap {
    pub fn load(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path).map_err(|e| {
            BridgeError::ConfigError(format!("cannot read pedal map {}: {e}", path.display()))
        })?;
        Self::parse(&text).map_err(|e| {
            BridgeError::ConfigError(format!("cannot parse pedal map {}: {e}", path.display()))
        })
    }

    pub fn parse(text: &str) -> std::result::Result<Self, String> {
        let mut rows = text
            .lines()
            .map(str::trim)
            .filter(|l| !l.is_empty() && !l.starts_with('#'));

        let header = rows.next().ok_or("the file has no header row")?;
        let speeds: Vec<f64> = header
            .split(',')
            .skip(1) // the corner cell is a label, not a speed
            .map(|s| s.trim().parse::<f64>().map_err(|e| format!("bad speed '{s}': {e}")))
            .collect::<std::result::Result<_, _>>()?;
        if speeds.len() < 2 {
            return Err("a map needs at least two speed columns to interpolate".into());
        }

        let mut pedals = Vec::new();
        let mut accel = Vec::new();
        for line in rows {
            let mut cells = line.split(',');
            let pedal = cells
                .next()
                .ok_or("a row with no pedal column")?
                .trim()
                .parse::<f64>()
                .map_err(|e| format!("bad pedal value: {e}"))?;
            let values: Vec<f64> = cells
                .map(|s| s.trim().parse::<f64>().map_err(|e| format!("bad cell '{s}': {e}")))
                .collect::<std::result::Result<_, _>>()?;
            if values.len() != speeds.len() {
                return Err(format!(
                    "pedal row {pedal} has {} cells against {} speed columns",
                    values.len(),
                    speeds.len()
                ));
            }
            pedals.push(pedal);
            accel.push(values);
        }
        if pedals.len() < 2 {
            return Err("a map needs at least two pedal rows to interpolate".into());
        }
        if !speeds.windows(2).all(|w| w[0] < w[1]) {
            return Err("speed columns must ascend".into());
        }
        if !pedals.windows(2).all(|w| w[0] < w[1]) {
            return Err("pedal rows must ascend".into());
        }
        Ok(Self { speeds, pedals, accel })
    }

    /// The acceleration this pedal row produces at `speed`, interpolated across speed and
    /// held flat outside the measured range.
    fn accel_at(&self, pedal_index: usize, speed: f64) -> f64 {
        let row = &self.accel[pedal_index];
        match self.speeds.iter().position(|&s| s >= speed) {
            None => *row.last().expect("rows are non-empty"),
            Some(0) => row[0],
            Some(i) => {
                let (s0, s1) = (self.speeds[i - 1], self.speeds[i]);
                let t = (speed - s0) / (s1 - s0);
                row[i - 1] + t * (row[i] - row[i - 1])
            }
        }
    }

    /// The pedal that produces `target` acceleration at `speed`.
    ///
    /// Returns the closest achievable pedal when the request is outside what the vehicle can
    /// do at this speed, which is the honest answer: asking for 3 m/s^2 at 20 m/s when the
    /// engine can only deliver 2.4 should give full throttle, not an error.
    ///
    /// `rising` says which way acceleration moves as the pedal opens: true for a throttle map
    /// (more pedal, more acceleration), false for a brake map (more pedal, more negative).
    pub fn pedal_for(&self, target: f64, speed: f64, rising: bool) -> f64 {
        let at = |i: usize| self.accel_at(i, speed);
        let last = self.pedals.len() - 1;

        // Outside the table's reach in either direction.
        let beyond_end = if rising { target >= at(last) } else { target <= at(last) };
        if beyond_end {
            return self.pedals[last];
        }
        let below_start = if rising { target <= at(0) } else { target >= at(0) };
        if below_start {
            return self.pedals[0];
        }

        for i in 1..=last {
            let (a0, a1) = (at(i - 1), at(i));
            let brackets = if rising {
                target >= a0 && target <= a1
            } else {
                target <= a0 && target >= a1
            };
            if brackets {
                let span = a1 - a0;
                // A flat pair carries no information about where between them to sit; take
                // the lower pedal, which is the smaller command.
                if span.abs() < f64::EPSILON {
                    return self.pedals[i - 1];
                }
                let t = (target - a0) / span;
                return self.pedals[i - 1] + t * (self.pedals[i] - self.pedals[i - 1]);
            }
        }

        // The table is not monotonic at this speed, so no pair brackets the request. Fall back
        // to whichever row comes closest rather than guessing between them.
        (0..=last)
            .min_by(|&a, &b| {
                (at(a) - target)
                    .abs()
                    .partial_cmp(&(at(b) - target).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|i| self.pedals[i])
            .unwrap_or(0.0)
    }

    /// Acceleration at the lowest pedal in the table, i.e. what the car does if left alone.
    pub fn passive_accel(&self, speed: f64) -> f64 {
        self.accel_at(0, speed)
    }
}

/// The pair of tables, and the decision of which one a request belongs to.
#[derive(Debug, Clone)]
pub struct LongitudinalCalibration {
    accel_map: PedalMap,
    brake_map: PedalMap,
}

/// What to send CARLA for one acceleration request.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PedalCommand {
    pub throttle: f32,
    pub brake: f32,
}

impl LongitudinalCalibration {
    pub fn load(accel_map: &Path, brake_map: &Path) -> Result<Self> {
        Ok(Self {
            accel_map: PedalMap::load(accel_map)?,
            brake_map: PedalMap::load(brake_map)?,
        })
    }

    /// Choose throttle or brake for a requested acceleration at the current speed.
    ///
    /// The dividing line is what the car does with no brake at all. CARLA's drag and engine
    /// braking are strong -- from 2.6 m/s^2 at rest to 8.8 m/s^2 at 24 m/s -- so a request
    /// for mild deceleration at speed is already satisfied by lifting off, and adding brake
    /// would overshoot it. Below that line the brake map decides how much.
    pub fn command_for(&self, requested_accel: f64, speed: f64) -> PedalCommand {
        let coasting = self.brake_map.passive_accel(speed);

        if requested_accel >= coasting {
            let throttle = self.accel_map.pedal_for(requested_accel, speed, true);
            PedalCommand { throttle: throttle.clamp(0.0, 1.0) as f32, brake: 0.0 }
        } else {
            let brake = self.brake_map.pedal_for(requested_accel, speed, false);
            PedalCommand { throttle: 0.0, brake: brake.clamp(0.0, 1.0) as f32 }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ACCEL: &str = "\
# comment
default,0.0,10.0,20.0
0.00,0.000,0.000,0.000
0.50,3.000,1.000,0.000
1.00,6.000,5.000,4.000
";

    const BRAKE: &str = "\
default,0.0,10.0,20.0
0.00,-2.000,-4.000,-8.000
0.50,-4.000,-6.000,-10.000
1.00,-6.000,-8.000,-12.000
";

    fn calibration() -> LongitudinalCalibration {
        LongitudinalCalibration {
            accel_map: PedalMap::parse(ACCEL).unwrap(),
            brake_map: PedalMap::parse(BRAKE).unwrap(),
        }
    }

    #[test]
    fn comments_and_headers_are_skipped() {
        let m = PedalMap::parse(ACCEL).unwrap();
        assert_eq!(m.speeds, vec![0.0, 10.0, 20.0]);
        assert_eq!(m.pedals, vec![0.0, 0.5, 1.0]);
    }

    #[test]
    fn a_ragged_row_is_rejected_rather_than_padded() {
        let bad = "default,0.0,10.0\n0.00,1.0\n0.50,1.0,2.0\n";
        assert!(PedalMap::parse(bad).is_err());
    }

    #[test]
    fn acceleration_interpolates_across_speed() {
        let m = PedalMap::parse(ACCEL).unwrap();
        assert!((m.accel_at(1, 5.0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn speeds_outside_the_table_hold_the_edge_value() {
        let m = PedalMap::parse(ACCEL).unwrap();
        assert!((m.accel_at(2, -5.0) - 6.0).abs() < 1e-9);
        assert!((m.accel_at(2, 99.0) - 4.0).abs() < 1e-9);
    }

    #[test]
    fn pedal_is_inverted_from_the_table() {
        let m = PedalMap::parse(ACCEL).unwrap();
        // At rest, 0.5 gives 3.0 and 1.0 gives 6.0, so 4.5 sits halfway between them.
        assert!((m.pedal_for(4.5, 0.0, true) - 0.75).abs() < 1e-9);
    }

    #[test]
    fn a_request_beyond_the_vehicle_saturates_rather_than_failing() {
        let m = PedalMap::parse(ACCEL).unwrap();
        assert_eq!(m.pedal_for(50.0, 0.0, true), 1.0);
        assert_eq!(m.pedal_for(-50.0, 0.0, true), 0.0);
    }

    /// The bug this module exists to fix: one constant for both pedals and every speed.
    #[test]
    fn the_same_request_gives_different_pedals_at_different_speeds() {
        let c = calibration();
        let at_rest = c.command_for(3.0, 0.0);
        let at_speed = c.command_for(3.0, 20.0);
        assert!((at_rest.throttle - 0.5).abs() < 1e-6, "got {at_rest:?}");
        assert!(at_speed.throttle > at_rest.throttle, "{at_speed:?} vs {at_rest:?}");
    }

    #[test]
    fn braking_and_throttle_are_never_commanded_together() {
        let c = calibration();
        for (accel, speed) in [(3.0, 0.0), (-1.0, 0.0), (-9.0, 10.0), (0.0, 20.0)] {
            let cmd = c.command_for(accel, speed);
            assert!(cmd.throttle == 0.0 || cmd.brake == 0.0, "{accel} at {speed}: {cmd:?}");
        }
    }

    /// Coasting already decelerates hard at speed, so a mild deceleration needs no brake.
    #[test]
    fn mild_deceleration_at_speed_coasts_instead_of_braking() {
        let c = calibration();
        let cmd = c.command_for(-2.0, 20.0); // coasting there is -8.0
        assert_eq!(cmd.brake, 0.0, "{cmd:?}");
    }

    #[test]
    fn firm_deceleration_reaches_the_brake() {
        let c = calibration();
        let cmd = c.command_for(-10.0, 20.0);
        assert!((cmd.brake - 0.5).abs() < 1e-6, "{cmd:?}");
        assert_eq!(cmd.throttle, 0.0);
    }
}
