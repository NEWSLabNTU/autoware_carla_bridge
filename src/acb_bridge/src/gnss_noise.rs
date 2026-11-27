//! The GNSS receiver's error (scensim issue 195 A9).
//!
//! CARLA's `sensor.other.gnss` reads the antenna's true position. A receiver does not: this
//! adds an RTK fix's error, east, north and up in metres, before the fix is published. The
//! model and its names are aib's (`aib.sensing.GnssNoise`), so both simulators' GNSS mean the
//! same thing: a bias drawn once per receiver from N(0, `bias_stddev_m`) on each axis, plus
//! white noise per fix from N(0, `horizontal_stddev_m`) east and north and
//! N(0, `vertical_stddev_m`) up. Not CARLA's own `noise_*` attributes: those are in degrees
//! and draw from CARLA's generator, which neither matches aib nor replays from our seed.
//!
//! The defaults (0.02, 0.04, 0 m) live here, not in the vehicle spec: a receiver default is
//! not worth moving every spec_hash. The parameters `gnss_noise_*` override them.

/// The radius the offset is turned into degrees with (WGS84's equatorial, CARLA's own). The
/// offsets are centimetres; the sphere's error on them is far below a millimetre.
pub const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// sensor_msgs/NavSatFix position_covariance_type values.
pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;

#[derive(Debug, Clone, PartialEq)]
pub struct GnssNoiseSettings {
    /// false: the antenna's true position and no covariance, as before A9.
    pub enabled: bool,
    pub horizontal_stddev_m: f64,
    pub vertical_stddev_m: f64,
    pub bias_stddev_m: f64,
    /// Some: the fixes replay; None: a fresh seed, drawn once and logged.
    pub seed: Option<u64>,
    /// The receiver's heading error, yaw, per fix (deg; scensim issue 195 L1b).
    pub heading_stddev_deg: f64,
    /// Its roll and pitch error, per fix (deg).
    pub attitude_stddev_deg: f64,
}

impl Default for GnssNoiseSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            horizontal_stddev_m: 0.02,
            vertical_stddev_m: 0.04,
            bias_stddev_m: 0.0,
            seed: None,
            heading_stddev_deg: 0.2,
            attitude_stddev_deg: 0.05,
        }
    }
}

impl GnssNoiseSettings {
    /// Refuse a negative or non-finite spread, naming its parameter.
    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [
            ("gnss_noise_horizontal_stddev_m", self.horizontal_stddev_m),
            ("gnss_noise_vertical_stddev_m", self.vertical_stddev_m),
            ("gnss_noise_bias_stddev_m", self.bias_stddev_m),
            ("gnss_noise_heading_stddev_deg", self.heading_stddev_deg),
            ("gnss_noise_attitude_stddev_deg", self.attitude_stddev_deg),
        ] {
            if !(v.is_finite() && v >= 0.0) {
                let unit = if name.ends_with("_deg") { "deg" } else { "m" };
                return Err(format!(
                    "{name} must be a finite number >= 0 ({unit}), is {v}"
                ));
            }
        }
        Ok(())
    }

    /// `gnss_noise_seed`: an integer >= 0, or -1 for a fresh one.
    pub fn seed_from_param(v: i64) -> Result<Option<u64>, String> {
        match v {
            -1 => Ok(None),
            v if v >= 0 => Ok(Some(v as u64)),
            v => Err(format!(
                "gnss_noise_seed must be >= 0, or -1 for a fresh seed, is {v}"
            )),
        }
    }

    /// These settings with the seed fixed: a fresh one drawn now when there is none, so
    /// every receiver this process makes (a respawned hero's too) draws the same stream.
    pub fn with_seed(&self) -> Self {
        let mut s = self.clone();
        if s.seed.is_none() {
            s.seed = Some(fresh_seed());
        }
        s
    }
}

/// A seed from the clock and the process, hashed: different per call and per process.
fn fresh_seed() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static CALLS: AtomicU64 = AtomicU64::new(0);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let mut state = nanos
        ^ (u64::from(std::process::id()) << 32)
        ^ CALLS
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_mul(0x9E37_79B9_7F4A_7C15);
    // within i64, so the log's number can be passed back as gnss_noise_seed
    splitmix64(&mut state) >> 1
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// The salt that seeds the orientation's generator from the receiver's seed: a second stream,
/// so the orientation draws leave a seed's position errors as they were before L1b.
const INS_SALT: u64 = 0x195A_11B5_0000_0001;

/// A standard normal draw from a SplitMix64 state (Box-Muller, the spare kept).
fn gauss(state: &mut u64, spare: &mut Option<f64>) -> f64 {
    if let Some(z) = spare.take() {
        return z;
    }
    // u1 in (0, 1]: the top 53 bits, never 0, so ln is finite
    let u1 = ((splitmix64(state) >> 11) + 1) as f64 / (1u64 << 53) as f64;
    let u2 = (splitmix64(state) >> 11) as f64 / (1u64 << 53) as f64;
    let r = (-2.0 * u1.ln()).sqrt();
    let t = std::f64::consts::TAU * u2;
    *spare = Some(r * t.sin());
    r * t.cos()
}

/// One receiver's error. A small generator of its own (SplitMix64, Box-Muller), so the
/// stream is fixed by the seed alone, whatever crate versions the build resolves.
pub struct GnssNoise {
    enabled: bool,
    seed: u64,
    state: u64,
    sigma: [f64; 3],
    bias_stddev: f64,
    bias: [f64; 3],
    spare: Option<f64>,
    /// The orientation's own generator (L1b), seeded from `seed` with INS_SALT
    ins_state: u64,
    ins_spare: Option<f64>,
    /// roll, pitch, yaw stddevs (rad)
    rot_sigma: [f64; 3],
}

impl GnssNoise {
    pub fn new(settings: &GnssNoiseSettings) -> Self {
        let seed = settings.seed.unwrap_or_else(fresh_seed);
        let h = settings.horizontal_stddev_m;
        let mut n = Self {
            enabled: settings.enabled,
            seed,
            state: seed,
            sigma: [h, h, settings.vertical_stddev_m],
            bias_stddev: settings.bias_stddev_m,
            bias: [0.0; 3],
            spare: None,
            ins_state: seed ^ INS_SALT,
            ins_spare: None,
            rot_sigma: [
                settings.attitude_stddev_deg.to_radians(),
                settings.attitude_stddev_deg.to_radians(),
                settings.heading_stddev_deg.to_radians(),
            ],
        };
        if n.enabled {
            let b = settings.bias_stddev_m;
            n.bias = [b * n.normal(), b * n.normal(), b * n.normal()];
        }
        n
    }

    pub fn seed(&self) -> u64 {
        self.seed
    }

    pub fn bias(&self) -> [f64; 3] {
        self.bias
    }

    /// A standard normal draw from the position's stream.
    fn normal(&mut self) -> f64 {
        gauss(&mut self.state, &mut self.spare)
    }

    /// The next fix's error, east, north, up (m).
    pub fn offset(&mut self) -> [f64; 3] {
        if !self.enabled {
            return [0.0; 3];
        }
        let mut o = self.bias;
        for (axis, v) in o.iter_mut().enumerate() {
            *v += self.sigma[axis] * self.normal();
        }
        o
    }

    /// The next fix's orientation error: (roll, pitch, yaw) in rad, the sensor's own frame.
    pub fn orientation_error(&mut self) -> [f64; 3] {
        if !self.enabled {
            return [0.0; 3];
        }
        let mut e = [0.0; 3];
        for (axis, v) in e.iter_mut().enumerate() {
            *v = self.rot_sigma[axis] * gauss(&mut self.ins_state, &mut self.ins_spare);
        }
        e
    }

    /// GnssInsOrientation's rmse_rotation_x, _y, _z (rad): the stddevs; disabled, zeros.
    pub fn rmse_rotation(&self) -> [f64; 3] {
        if self.enabled {
            self.rot_sigma
        } else {
            [0.0; 3]
        }
    }

    /// (NavSatFix position_covariance, its type): the white noise's and the bias's variance
    /// on the diagonal, known; disabled, zeros of unknown type.
    pub fn covariance(&self) -> ([f64; 9], u8) {
        if !self.enabled {
            return ([0.0; 9], COVARIANCE_TYPE_UNKNOWN);
        }
        let b2 = self.bias_stddev * self.bias_stddev;
        let mut c = [0.0; 9];
        c[0] = self.sigma[0] * self.sigma[0] + b2;
        c[4] = self.sigma[1] * self.sigma[1] + b2;
        c[8] = self.sigma[2] * self.sigma[2] + b2;
        (c, COVARIANCE_TYPE_DIAGONAL_KNOWN)
    }

    /// A one-line description for the log: enough to replay the run (its seed) and to see
    /// this run's bias.
    pub fn describe(&self) -> String {
        if self.enabled {
            let [e, n, u] = self.bias();
            format!(
                "error east/north {} m, up {} m, bias {} m (this run's: {e:.4}, {n:.4}, \
                 {u:.4}), heading {} deg, attitude {} deg, seed {}",
                self.sigma[0],
                self.sigma[2],
                self.bias_stddev,
                self.rot_sigma[2].to_degrees(),
                self.rot_sigma[0].to_degrees(),
                self.seed()
            )
        } else {
            "no error (gnss_noise_enabled false): the antenna's true position".to_string()
        }
    }
}

/// GNSS periods the orientation goes out before the first fix (scensim issue 195 L1c).
pub const INS_HOLD_PERIODS: u32 = 2;

/// The fix held until the orientation has gone out INS_HOLD_PERIODS times. gnss_poser
/// (autoware_core 1.5.0) starts from a default GnssInsOrientationStamped, an identity quaternion,
/// and publishes every fix with the last orientation it received, so a fix ahead of any
/// orientation became a GNSS pose at yaw 0. Publish order alone is no delivery order across two
/// DDS topics; the hold gives the orientation two periods' margin. Once per receiver: a teleport
/// (a scenario's reset) does not hold it again, gnss_poser keeping its last orientation.
#[derive(Debug, Default)]
pub struct FixHold {
    orientations: u32,
    fix_reported: bool,
}

impl FixHold {
    /// Count one published orientation; true when this period's fix may go out.
    pub fn orientation_published(&mut self) -> bool {
        self.orientations = self.orientations.saturating_add(1);
        self.orientations > INS_HOLD_PERIODS
    }

    /// True the first time it is asked (for the log line), false after.
    pub fn first_fix(&mut self) -> bool {
        !std::mem::replace(&mut self.fix_reported, true)
    }
}

/// The receiver's orientation in the map (w, x, y, z): the GNSS sensor's CARLA rotation (deg,
/// its world transform at the measurement) in ROS's frame, turned by `error` (roll, pitch, yaw
/// in rad, the sensor's own frame). CARLA's world is the map but for a translation.
pub fn ins_quaternion(roll_deg: f64, pitch_deg: f64, yaw_deg: f64, error: [f64; 3]) -> [f64; 4] {
    let q =
        crate::coordinate_conversion::carla_euler_to_ros_quaternion(roll_deg, pitch_deg, yaw_deg);
    let q = nalgebra::UnitQuaternion::from_quaternion(q)
        * nalgebra::UnitQuaternion::from_euler_angles(error[0], error[1], error[2]);
    [q.w, q.i, q.j, q.k]
}

/// A fix (degrees, m) moved by `offset` (east, north, up in m), on a sphere of
/// EARTH_RADIUS_M. `lat` is the published latitude, north positive.
pub fn apply(lat: f64, lon: f64, alt: f64, offset: [f64; 3]) -> (f64, f64, f64) {
    let [e, n, u] = offset;
    let lat_out = if n == 0.0 {
        lat
    } else {
        lat + (n / EARTH_RADIUS_M).to_degrees()
    };
    let lon_out = if e == 0.0 {
        lon
    } else {
        lon + (e / (EARTH_RADIUS_M * lat.to_radians().cos())).to_degrees()
    };
    (lat_out, lon_out, alt + u)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings() -> GnssNoiseSettings {
        GnssNoiseSettings::default()
    }

    fn draws(s: GnssNoiseSettings, n: usize) -> Vec<[f64; 3]> {
        let mut noise = GnssNoise::new(&s);
        (0..n).map(|_| noise.offset()).collect()
    }

    fn mean_sd(v: &[[f64; 3]], axis: usize) -> (f64, f64) {
        let n = v.len() as f64;
        let m = v.iter().map(|d| d[axis]).sum::<f64>() / n;
        let var = v.iter().map(|d| (d[axis] - m).powi(2)).sum::<f64>() / n;
        (m, var.sqrt())
    }

    #[test]
    fn the_defaults_are_rtk_like() {
        let s = settings();
        assert!(s.enabled);
        assert_eq!(
            (
                s.horizontal_stddev_m,
                s.vertical_stddev_m,
                s.bias_stddev_m,
                s.seed
            ),
            (0.02, 0.04, 0.0, None)
        );
        assert!(s.validate().is_ok());
    }

    #[test]
    fn bad_values_are_refused_by_name() {
        for (name, s) in [
            (
                "gnss_noise_horizontal_stddev_m",
                GnssNoiseSettings {
                    horizontal_stddev_m: -0.1,
                    ..settings()
                },
            ),
            (
                "gnss_noise_vertical_stddev_m",
                GnssNoiseSettings {
                    vertical_stddev_m: f64::NAN,
                    ..settings()
                },
            ),
            (
                "gnss_noise_bias_stddev_m",
                GnssNoiseSettings {
                    bias_stddev_m: -1.0,
                    ..settings()
                },
            ),
        ] {
            let e = s.validate().unwrap_err();
            assert!(e.contains(name), "{e}");
        }
        assert_eq!(GnssNoiseSettings::seed_from_param(-1).unwrap(), None);
        assert_eq!(GnssNoiseSettings::seed_from_param(7).unwrap(), Some(7));
        assert!(GnssNoiseSettings::seed_from_param(-2)
            .unwrap_err()
            .contains("gnss_noise_seed"));
    }

    #[test]
    fn white_noise_has_the_configured_spread_and_no_mean() {
        let d = draws(
            GnssNoiseSettings {
                seed: Some(7),
                ..settings()
            },
            20000,
        );
        let se = |s: f64| s / (20000f64).sqrt();
        for (axis, sigma) in [(0, 0.02), (1, 0.02), (2, 0.04)] {
            let (m, sd) = mean_sd(&d, axis);
            assert!((sd - sigma).abs() < 0.02 * sigma, "axis {axis}: sd {sd}");
            assert!(m.abs() < 4.0 * se(sigma), "axis {axis}: mean {m}");
        }
        // independent axes
        let (m0, s0) = mean_sd(&d, 0);
        let (m1, s1) = mean_sd(&d, 1);
        let c = d.iter().map(|v| (v[0] - m0) * (v[1] - m1)).sum::<f64>() / 20000.0 / (s0 * s1);
        assert!(c.abs() < 0.05, "corr {c}");
    }

    #[test]
    fn a_bias_is_drawn_once_per_receiver() {
        let s = GnssNoiseSettings {
            seed: Some(3),
            horizontal_stddev_m: 0.0,
            vertical_stddev_m: 0.0,
            bias_stddev_m: 0.5,
            ..settings()
        };
        let d = draws(s.clone(), 100);
        assert!(d.iter().all(|v| *v == d[0]));
        assert!(d[0].iter().any(|x| *x != 0.0));
        assert_eq!(d[0], GnssNoise::new(&s).bias());
        let biases: Vec<[f64; 3]> = (0..2000)
            .map(|k| {
                GnssNoise::new(&GnssNoiseSettings {
                    seed: Some(k),
                    bias_stddev_m: 0.5,
                    ..settings()
                })
                .bias()
            })
            .collect();
        for axis in 0..3 {
            let (_, sd) = mean_sd(&biases, axis);
            assert!((sd - 0.5).abs() < 0.05, "axis {axis}: {sd}");
        }
    }

    #[test]
    fn the_same_seed_gives_the_same_fixes() {
        let s = |k| GnssNoiseSettings {
            seed: Some(k),
            bias_stddev_m: 0.1,
            ..settings()
        };
        assert_eq!(draws(s(42), 50), draws(s(42), 50));
        assert_ne!(draws(s(42), 50), draws(s(43), 50));
    }

    #[test]
    fn no_seed_draws_a_fresh_one_and_names_it() {
        let a = GnssNoise::new(&settings());
        let b = GnssNoise::new(&settings());
        assert_ne!(a.seed(), b.seed());
        let replay = GnssNoiseSettings {
            seed: Some(a.seed()),
            ..settings()
        };
        assert_eq!(draws(replay.clone(), 5), draws(replay, 5));
    }

    #[test]
    fn disabled_is_the_truth() {
        let s = GnssNoiseSettings {
            enabled: false,
            seed: Some(1),
            bias_stddev_m: 1.0,
            ..settings()
        };
        assert!(draws(s.clone(), 10).iter().all(|v| *v == [0.0; 3]));
        assert_eq!(GnssNoise::new(&s).covariance(), ([0.0; 9], 0));
    }

    #[test]
    fn the_covariance_is_diagonal_and_known() {
        let (c, kind) = GnssNoise::new(&GnssNoiseSettings {
            seed: Some(0),
            bias_stddev_m: 0.01,
            ..settings()
        })
        .covariance();
        assert_eq!(kind, 2); // NavSatFix COVARIANCE_TYPE_DIAGONAL_KNOWN
        let close = |a: f64, b: f64| (a - b).abs() < 1e-15;
        assert!(close(c[0], 0.02f64.powi(2) + 0.01f64.powi(2)));
        assert!(close(c[4], 0.02f64.powi(2) + 0.01f64.powi(2)));
        assert!(close(c[8], 0.04f64.powi(2) + 0.01f64.powi(2)));
        for i in [1, 2, 3, 5, 6, 7] {
            assert_eq!(c[i], 0.0);
        }
    }

    #[test]
    fn the_offset_moves_the_fix_east_north_up() {
        let (lat, lon, alt) = (48.0, 11.0, 500.0);
        let (la, lo, al) = apply(lat, lon, alt, [0.0; 3]);
        assert_eq!((la, lo, al), (lat, lon, alt));
        // one metre north: 1/R rad of latitude
        let (la, lo, _) = apply(lat, lon, alt, [0.0, 1.0, 0.0]);
        assert!((la - lat - (1.0 / EARTH_RADIUS_M).to_degrees()).abs() < 1e-12);
        assert_eq!(lo, lon);
        // one metre east: 1/(R cos lat) rad of longitude
        let (la, lo, _) = apply(lat, lon, alt, [1.0, 0.0, 0.0]);
        let want = (1.0 / (EARTH_RADIUS_M * lat.to_radians().cos())).to_degrees();
        assert!((lo - lon - want).abs() < 1e-12);
        assert_eq!(la, lat);
        let (_, _, al) = apply(lat, lon, alt, [0.0, 0.0, 0.5]);
        assert_eq!(al, 500.5);
    }

    // --- L1b (scensim issue 195): the receiver's heading and attitude ---

    const DEG: f64 = std::f64::consts::PI / 180.0;

    fn ins_draws(s: GnssNoiseSettings, n: usize) -> Vec<[f64; 3]> {
        let mut noise = GnssNoise::new(&s);
        (0..n)
            .map(|_| {
                noise.offset();
                noise.orientation_error()
            })
            .collect()
    }

    #[test]
    fn the_ins_defaults_are_a_dual_antenna_receivers() {
        let s = settings();
        assert_eq!((s.heading_stddev_deg, s.attitude_stddev_deg), (0.2, 0.05));
    }

    #[test]
    fn bad_ins_values_are_refused_by_name() {
        for (name, s) in [
            (
                "gnss_noise_heading_stddev_deg",
                GnssNoiseSettings {
                    heading_stddev_deg: -0.1,
                    ..settings()
                },
            ),
            (
                "gnss_noise_attitude_stddev_deg",
                GnssNoiseSettings {
                    attitude_stddev_deg: f64::INFINITY,
                    ..settings()
                },
            ),
        ] {
            let e = s.validate().unwrap_err();
            assert!(e.contains(name), "{e}");
        }
    }

    #[test]
    fn a_still_cars_orientation_errors_have_the_configured_spread() {
        let d = ins_draws(
            GnssNoiseSettings {
                seed: Some(11),
                ..settings()
            },
            20000,
        );
        for (axis, sigma) in [(0, 0.05 * DEG), (1, 0.05 * DEG), (2, 0.2 * DEG)] {
            let (m, sd) = mean_sd(&d, axis);
            assert!((sd - sigma).abs() < 0.02 * sigma, "axis {axis}: sd {sd}");
            assert!(
                m.abs() < 4.0 * sigma / (20000f64).sqrt(),
                "axis {axis}: mean {m}"
            );
        }
    }

    #[test]
    fn the_same_seed_gives_the_same_headings() {
        let s = |k| GnssNoiseSettings {
            seed: Some(k),
            ..settings()
        };
        assert_eq!(ins_draws(s(5), 30), ins_draws(s(5), 30));
        assert_ne!(ins_draws(s(5), 30), ins_draws(s(6), 30));
    }

    #[test]
    fn the_heading_leaves_a_seeds_position_errors_as_they_were() {
        let s = GnssNoiseSettings {
            seed: Some(0),
            bias_stddev_m: 0.1,
            ..settings()
        };
        let plain = draws(s.clone(), 20);
        let mut noise = GnssNoise::new(&s);
        let with_ins: Vec<[f64; 3]> = (0..20)
            .map(|_| {
                let o = noise.offset();
                noise.orientation_error();
                o
            })
            .collect();
        assert_eq!(plain, with_ins);
    }

    #[test]
    fn disabled_is_the_exact_orientation() {
        let s = GnssNoiseSettings {
            enabled: false,
            seed: Some(1),
            ..settings()
        };
        assert!(ins_draws(s.clone(), 10).iter().all(|v| *v == [0.0; 3]));
        assert_eq!(GnssNoise::new(&s).rmse_rotation(), [0.0; 3]);
    }

    #[test]
    fn the_rmse_fields_carry_the_stddevs_in_rad() {
        let r = GnssNoise::new(&GnssNoiseSettings {
            seed: Some(0),
            ..settings()
        })
        .rmse_rotation();
        for (got, want) in r.iter().zip([0.05 * DEG, 0.05 * DEG, 0.2 * DEG]) {
            assert!((got - want).abs() < 1e-15, "{got} {want}");
        }
    }

    #[test]
    fn the_orientation_is_the_sensors_in_the_map_turned_by_the_error() {
        // CARLA yaw +30 deg (clockwise from above, left-handed) is ROS yaw -30 deg
        let exact = ins_quaternion(0.0, 0.0, 30.0, [0.0; 3]);
        let want = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, -30.0 * DEG);
        let got = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            exact[0], exact[1], exact[2], exact[3],
        ));
        assert!(got.angle_to(&want) < 1e-6, "{exact:?}");
        // a yaw error of 0.01 rad turns it about its own z
        let turned = ins_quaternion(0.0, 0.0, 30.0, [0.0, 0.0, 0.01]);
        let t = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            turned[0], turned[1], turned[2], turned[3],
        ));
        let (_, _, yaw) = t.euler_angles();
        assert!((yaw - (-30.0 * DEG + 0.01)).abs() < 1e-6, "{yaw}");
    }

    // --- L1c (scensim issue 195): the fix held until the orientation has gone out ---

    #[test]
    fn the_first_fix_goes_out_two_orientation_periods_after_the_first_orientation() {
        let mut hold = FixHold::default();
        let sent: Vec<bool> = (0..5).map(|_| hold.orientation_published()).collect();
        assert_eq!(sent, [false, false, true, true, true]);
        assert_eq!(INS_HOLD_PERIODS, 2);
    }

    #[test]
    fn the_first_fix_is_reported_once() {
        let mut hold = FixHold::default();
        let firsts: Vec<bool> = (0..5)
            .map(|_| {
                let send = hold.orientation_published();
                send && hold.first_fix()
            })
            .collect();
        assert_eq!(firsts, [false, false, true, false, false]);
    }
}
