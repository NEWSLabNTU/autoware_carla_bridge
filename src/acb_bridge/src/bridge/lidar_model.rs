//! What a lidar point carries besides its position: its ring and its intensity.
//!
//! **Ring.** CARLA spaces a lidar's beams evenly from `upper_fov` (beam 0) down to
//! `lower_fov` (beam `channels - 1`), so a point's beam is its elevation in the sensor frame,
//! exactly (measured on 0.9.16: beam 1 at 14.69 deg for 128 beams over [-25, 15]). The beam
//! comes from that, not from the measurement's per-channel point counts: walked over those,
//! every point was published as channel 0 (roadmap 074 S5, swb 118).
//!
//! **Intensity.** The ray-cast lidar's intensity is only the atmosphere's attenuation by range
//! (`exp(-rate d)`, 1.0 at the spec's rate 0): every point 255, road and car alike, and a
//! detector that reads intensity (TransFusion) sees nothing. The semantic lidar says what
//! each ray hit (its tag) and at what incidence, so its intensity is the spec's reflectance of
//! the tag times the cosine of the incidence -- a Lambertian surface seen by a lidar that
//! reports reflectivity (carla_sensors.yaml's `intensity.reflectance`, by tag id).

use std::collections::HashMap;

/// A lidar's beams: CARLA's `upper_fov`, `lower_fov` (deg) and `channels`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Beams {
    upper_deg: f32,
    lower_deg: f32,
    channels: u16,
}

impl Beams {
    /// From the blueprint's attributes; CARLA's defaults (10, -30, 32) for any not given.
    pub fn from_parameters(p: &HashMap<String, String>) -> Self {
        let get = |k: &str, d: f32| p.get(k).and_then(|v| v.parse().ok()).unwrap_or(d);
        Self {
            upper_deg: get("upper_fov", 10.0),
            lower_deg: get("lower_fov", -30.0),
            channels: get("channels", 32.0).max(1.0) as u16,
        }
    }

    /// The beam of a point at `(x, y, z)` in the sensor frame (either handedness): 0 the top.
    pub fn ring(&self, x: f32, y: f32, z: f32) -> u16 {
        if self.channels < 2 || self.upper_deg <= self.lower_deg {
            return 0;
        }
        let elevation = z.atan2(x.hypot(y)).to_degrees();
        let step = (self.upper_deg - self.lower_deg) / f32::from(self.channels - 1);
        let beam = ((self.upper_deg - elevation) / step).round();
        beam.clamp(0.0, f32::from(self.channels - 1)) as u16
    }
}

/// A semantic lidar's intensity model: the reflectance (0-255) by CARLA semantic tag id.
#[derive(Debug, Clone, PartialEq)]
pub struct Reflectance(Vec<f32>);

impl Reflectance {
    pub fn new(by_tag: Vec<f32>) -> Result<Self, String> {
        if by_tag.is_empty() {
            return Err("intensity.reflectance is empty".into());
        }
        if let Some(v) = by_tag.iter().find(|v| !(0.0..=255.0).contains(*v)) {
            return Err(format!("intensity.reflectance {v} is not in 0-255"));
        }
        Ok(Self(by_tag))
    }

    /// A point's intensity: its tag's reflectance times the cosine of its incidence. A tag
    /// the table does not reach reflects nothing.
    pub fn intensity(&self, tag: u32, cos_incidence: f32) -> u8 {
        let r = self.0.get(tag as usize).copied().unwrap_or(0.0);
        (r * cos_incidence.abs().min(1.0)).round().clamp(0.0, 255.0) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn beams() -> Beams {
        let p: HashMap<String, String> = [
            ("upper_fov", "15.0"),
            ("lower_fov", "-25.0"),
            ("channels", "128"),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
        Beams::from_parameters(&p)
    }

    fn at(elevation_deg: f32) -> (f32, f32, f32) {
        let e = elevation_deg.to_radians();
        (
            20.0 * e.cos(),
            3.0,
            20.0 * e.sin() * (1.0 + (3.0f32 / 20.0).powi(2)).sqrt(),
        )
    }

    #[test]
    fn the_beam_is_the_elevation_top_first() {
        let b = beams();
        // CARLA 0.9.16's measured beam elevations (128 over [-25, 15])
        for (elev, ring) in [(15.0, 0), (14.69, 1), (-5.16, 64), (-25.0, 127)] {
            let (x, y, z) = at(elev);
            assert_eq!(b.ring(x, y, z), ring, "{elev} deg");
            assert_eq!(b.ring(x, -y, z), ring, "either handedness");
        }
        let (x, y, z) = at(40.0);
        assert_eq!(b.ring(x, y, z), 0, "above the top beam clamps");
    }

    #[test]
    fn carlas_defaults_without_attributes() {
        let b = Beams::from_parameters(&HashMap::new());
        assert_eq!(
            b,
            Beams {
                upper_deg: 10.0,
                lower_deg: -30.0,
                channels: 32
            }
        );
    }

    #[test]
    fn intensity_is_the_tags_reflectance_by_the_incidence() {
        let mut t = vec![30.0; 29];
        t[1] = 5.0; // road
        t[14] = 90.0; // car
        let r = Reflectance::new(t).unwrap();
        assert_eq!(r.intensity(14, 1.0), 90);
        assert_eq!(r.intensity(14, 0.5), 45);
        assert_eq!(r.intensity(14, -0.5), 45, "a back face reflects as well");
        assert_eq!(r.intensity(1, 0.3), 2);
        assert_eq!(r.intensity(200, 1.0), 0, "past the table");
        assert!(Reflectance::new(vec![300.0]).is_err());
        assert!(Reflectance::new(vec![]).is_err());
    }
}
