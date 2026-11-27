//! The map's projector, inverted: a map point (x east, y north, metres) to the latitude and
//! longitude Autoware's own projector maps back to that point (scensim issue 195).
//!
//! The kit's gnss_poser projects every fix with the projector the map declares
//! (`map_projector_info.yaml`). A fix made any other way lands elsewhere: CARLA's geolocation is
//! spherical, and through the map's WGS84 transverse Mercator it comes back 0.82 m off at
//! Town01's (190.8, -130.1), growing with the distance from the origin (issue 195). So the bridge
//! makes the fix from the GNSS sensor's world position, through the exact inverse of that same
//! projector.
//!
//! Supported, as Autoware 1.5.0's map_projection_loader reads them:
//! - `TransverseMercator`: `map_origin` (latitude, longitude; the origin maps to (0, 0)) and
//!   `scale_factor` (0.9996 when the file omits it, the loader's default);
//! - `MGRS`: `mgrs_grid` (a 100 km square, e.g. `51RUH`); map coordinates are UTM's, less the
//!   square's south-west corner.
//!
//! `LocalCartesianUTM` and `Local` are refused by name: no fix can be made for them here.

/// The map's declared projector.
#[derive(Debug, Clone, PartialEq)]
pub enum MapProjector {
    TransverseMercator { lat0: f64, lon0: f64, alt0: f64, k0: f64 },
    Mgrs { zone: u32, north: bool, e0: f64, n0: f64 },
}

/// Autoware 1.5.0's map_projection_loader: a TransverseMercator without `scale_factor`.
pub const DEFAULT_SCALE: f64 = 0.9996;

impl MapProjector {
    /// From the fields of `map_projector_info.yaml`.
    pub fn from_fields(
        projector_type: &str,
        origin: Option<(f64, f64, f64)>,
        scale_factor: Option<f64>,
        mgrs_grid: Option<&str>,
    ) -> Result<Self, String> {
        let _ = (projector_type, origin, scale_factor, mgrs_grid);
        Err("not implemented".into())
    }

    /// From the text of `map_projector_info.yaml`.
    pub fn from_yaml(text: &str) -> Result<Self, String> {
        let _ = text;
        Err("not implemented".into())
    }

    /// The latitude and longitude (degrees) the projector maps to map (x, y).
    pub fn lat_lon(&self, x: f64, y: f64) -> (f64, f64) {
        let _ = (x, y);
        (f64::NAN, f64::NAN)
    }

    /// The altitude of a point at map z: the origin's altitude plus z.
    pub fn altitude(&self, z: f64) -> f64 {
        z
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Metres between two latitude/longitude points (small distances).
    fn metres(a: (f64, f64), b: (f64, f64)) -> f64 {
        let r = 6_378_137.0_f64;
        let dn = (a.0 - b.0).to_radians() * r;
        let de = (a.1 - b.1).to_radians() * r * a.0.to_radians().cos();
        (dn * dn + de * de).sqrt()
    }

    fn projector(p: &serde_json::Value) -> MapProjector {
        let origin = p.get("map_origin").map(|o| {
            (
                o["latitude"].as_f64().unwrap(),
                o["longitude"].as_f64().unwrap(),
                o.get("altitude").and_then(|a| a.as_f64()).unwrap_or(0.0),
            )
        });
        MapProjector::from_fields(
            p["projector_type"].as_str().unwrap(),
            origin,
            p.get("scale_factor").and_then(|s| s.as_f64()),
            p.get("mgrs_grid").and_then(|g| g.as_str()),
        )
        .unwrap()
    }

    /// The vectors shared with aib (scensim 118, NAS `home/2026-11-27 GNSS projector test
    /// vectors (118)/`): PROJ's tmerc and UTM, independent of either bridge's code.
    #[test]
    fn every_shared_vector_inverts_within_a_centimetre() {
        let v: serde_json::Value =
            serde_json::from_str(include_str!("../tests/data/gnss_projector_vectors.json"))
                .unwrap();
        let tol = v["tolerance_m"].as_f64().unwrap();
        let mut worst = (0.0_f64, String::new());
        for case in v["cases"].as_array().unwrap() {
            let p = projector(&case["projector"]);
            for pt in case["points"].as_array().unwrap() {
                let (x, y) = (pt["x"].as_f64().unwrap(), pt["y"].as_f64().unwrap());
                let want = (pt["lat"].as_f64().unwrap(), pt["lon"].as_f64().unwrap());
                let d = metres(p.lat_lon(x, y), want);
                assert!(d.is_finite(), "{}: ({x}, {y}) gave no fix", case["name"]);
                if d > worst.0 {
                    worst = (d, format!("{} ({x}, {y})", case["name"]));
                }
            }
        }
        assert!(worst.0 <= tol, "worst {:.4} m at {}", worst.0, worst.1);
    }

    #[test]
    fn a_carla_towns_file_reads_as_tm_at_zero_with_its_declared_scale() {
        let text = "projector_type: TransverseMercator\nvertical_datum: WGS84\nmap_origin:\n  \
                    latitude: 0.0\n  longitude: 0.0\n  altitude: 0.0\nscale_factor: 1.0\n";
        let p = MapProjector::from_yaml(text).unwrap();
        assert_eq!(p, MapProjector::TransverseMercator { lat0: 0.0, lon0: 0.0, alt0: 0.0, k0: 1.0 });
    }

    #[test]
    fn a_file_without_scale_factor_takes_the_loaders_default() {
        let text = "# a course fixture\nprojector_type: TransverseMercator\nvertical_datum: WGS84\n\
                    map_origin:\n  latitude: 36.0\n  longitude: 140.0\n  altitude: 0.0\n";
        match MapProjector::from_yaml(text).unwrap() {
            MapProjector::TransverseMercator { k0, lat0, lon0, .. } => {
                assert_eq!((k0, lat0, lon0), (DEFAULT_SCALE, 36.0, 140.0))
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn an_mgrs_file_reads_its_square() {
        let p = MapProjector::from_yaml("projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 51RUH\n")
            .unwrap();
        assert_eq!(p, MapProjector::Mgrs { zone: 51, north: true, e0: 300_000.0, n0: 2_700_000.0 });
    }

    #[test]
    fn projectors_without_a_geographic_inverse_are_refused_by_name() {
        for t in ["Local", "LocalCartesianUTM", "Mercator"] {
            let e = MapProjector::from_yaml(&format!("projector_type: {t}\n")).unwrap_err();
            assert!(e.contains(t), "{e}");
        }
        assert!(MapProjector::from_yaml("vertical_datum: WGS84\n").is_err());
    }

    /// Issue 195's measurement: on Town01 (TM at 0, 0, scale 1.0) the point (190.8, -130.1)
    /// must come back from Autoware's projector as itself, where CARLA's spherical fix came back
    /// at (190.876, -129.281). The vector file covers the general case; this names the one seen.
    #[test]
    fn town01s_measured_point_is_not_the_spherical_fix() {
        let p = MapProjector::TransverseMercator { lat0: 0.0, lon0: 0.0, alt0: 0.0, k0: 1.0 };
        let (lat, lon) = p.lat_lon(190.8, -130.1);
        // pyproj 3.3.0: +proj=tmerc +lat_0=0 +lon_0=0 +k=1 +ellps=WGS84, inverse of (190.8, -130.1)
        assert!(metres((lat, lon), (-0.0011765846891, 0.0017139855622)) < 0.01, "{lat} {lon}");
        // CARLA's own lat/lon for the same place is 0.82 m from it
        assert!(metres((lat, lon), (-0.001169175909, 0.001714671458)) > 0.8);
    }
}
