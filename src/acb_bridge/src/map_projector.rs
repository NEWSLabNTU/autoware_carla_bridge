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
    TransverseMercator {
        lat0: f64,
        lon0: f64,
        alt0: f64,
        k0: f64,
    },
    Mgrs {
        zone: u32,
        north: bool,
        e0: f64,
        n0: f64,
    },
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
        match projector_type {
            "TransverseMercator" => {
                let (lat0, lon0, alt0) = origin.ok_or(
                    "map_projector_info: a TransverseMercator projector without map_origin",
                )?;
                let k0 = scale_factor.unwrap_or(DEFAULT_SCALE);
                if k0.is_nan() || k0 <= 0.0 || !lat0.is_finite() || !lon0.is_finite() {
                    return Err(format!(
                        "map_projector_info: TransverseMercator at ({lat0}, {lon0}), scale {k0}"
                    ));
                }
                Ok(Self::TransverseMercator {
                    lat0,
                    lon0,
                    alt0,
                    k0,
                })
            }
            "MGRS" => {
                let grid =
                    mgrs_grid.ok_or("map_projector_info: an MGRS projector without mgrs_grid")?;
                mgrs_square(grid)
            }
            other => Err(format!(
                "map_projector_info: projector_type {other}: the bridge makes a GNSS fix only for \
                 TransverseMercator and MGRS maps (no geographic inverse for {other})"
            )),
        }
    }

    /// From the text of `map_projector_info.yaml` (its flat keys and `map_origin`'s).
    pub fn from_yaml(text: &str) -> Result<Self, String> {
        let mut top = std::collections::HashMap::new();
        let mut origin = std::collections::HashMap::new();
        let mut in_origin = false;
        for raw in text.lines() {
            let line = raw.split('#').next().unwrap_or("").trim_end();
            if line.trim().is_empty() {
                continue;
            }
            let indented = line.starts_with(' ') || line.starts_with('\t');
            let Some((k, v)) = line.trim().split_once(':') else {
                continue;
            };
            let (k, v) = (k.trim(), v.trim().trim_matches('"').trim_matches('\''));
            if !indented {
                in_origin = k == "map_origin";
                if !in_origin {
                    top.insert(k.to_string(), v.to_string());
                }
            } else if in_origin {
                origin.insert(k.to_string(), v.to_string());
            }
        }
        let num = |m: &std::collections::HashMap<String, String>,
                   k: &str|
         -> Result<Option<f64>, String> {
            m.get(k)
                .map(|v| {
                    v.parse::<f64>()
                        .map_err(|e| format!("map_projector_info: {k}: {v}: {e}"))
                })
                .transpose()
        };
        let projector_type = top
            .get("projector_type")
            .ok_or("map_projector_info: no projector_type")?
            .clone();
        let origin = match (num(&origin, "latitude")?, num(&origin, "longitude")?) {
            (Some(lat), Some(lon)) => Some((lat, lon, num(&origin, "altitude")?.unwrap_or(0.0))),
            _ => None,
        };
        Self::from_fields(
            &projector_type,
            origin,
            num(&top, "scale_factor")?,
            top.get("mgrs_grid").map(String::as_str),
        )
    }

    /// The latitude and longitude (degrees) the projector maps to map (x, y).
    pub fn lat_lon(&self, x: f64, y: f64) -> (f64, f64) {
        match *self {
            Self::TransverseMercator { lat0, lon0, k0, .. } => {
                let (_, n0) = tm_forward(lat0, 0.0, k0);
                tm_inverse(x, y + n0, lon0, k0)
            }
            Self::Mgrs {
                zone,
                north,
                e0,
                n0,
            } => {
                let lon0 = zone as f64 * 6.0 - 183.0;
                let northing = n0 + y - if north { 0.0 } else { 10_000_000.0 };
                tm_inverse(e0 + x - 500_000.0, northing, lon0, 0.9996)
            }
        }
    }

    /// The altitude of a point at map z: the origin's altitude plus z.
    pub fn altitude(&self, z: f64) -> f64 {
        match *self {
            Self::TransverseMercator { alt0, .. } => alt0 + z,
            Self::Mgrs { .. } => z,
        }
    }
}

/// What the GNSS fix is made with: the map's projector and where CARLA's world origin sits in
/// the map frame (`map_origin_path`).
#[derive(Debug, Clone)]
pub struct GnssGeo {
    pub projector: MapProjector,
    pub map_origin: crate::coordinate_conversion::MapOrigin,
}

/// CARLA's own convention, for a bridge run with no map (`map_projector_path` empty): a
/// transverse Mercator at (0, 0), scale 1.0, as every CARLA town's map declares.
pub const CARLA_DEFAULT: MapProjector = MapProjector::TransverseMercator {
    lat0: 0.0,
    lon0: 0.0,
    alt0: 0.0,
    k0: 1.0,
};

impl GnssGeo {
    /// From `map_projector_path` (the map dir's `map_projector_info.yaml`) and the map origin.
    /// Empty or `none`: [`CARLA_DEFAULT`]; a path that does not exist or does not parse is an
    /// error, since a wrong projector puts every fix metres off on a run that looks fine.
    pub fn load(
        path: &str,
        map_origin: crate::coordinate_conversion::MapOrigin,
    ) -> Result<Self, String> {
        if path.is_empty() || path == "none" {
            return Ok(Self {
                projector: CARLA_DEFAULT,
                map_origin,
            });
        }
        let text =
            std::fs::read_to_string(path).map_err(|e| format!("map_projector_path {path}: {e}"))?;
        let projector = MapProjector::from_yaml(&text).map_err(|e| format!("{path}: {e}"))?;
        Ok(Self {
            projector,
            map_origin,
        })
    }

    /// The fix (latitude, longitude in degrees, altitude in metres) for a GNSS sensor at
    /// `carla_location` (CARLA's frame) with the receiver's error `offset_enu` (east, north, up,
    /// in metres, in the map's frame) added.
    pub fn fix(&self, carla_location: [f64; 3], offset_enu: [f64; 3]) -> (f64, f64, f64) {
        let [x, y, z] = carla_location;
        let m = self
            .map_origin
            .carla_to_map(&nalgebra::Vector3::new(x, y, z));
        let [e, n, u] = offset_enu;
        let (lat, lon) = self.projector.lat_lon(m.x + e, m.y + n);
        (lat, lon, self.projector.altitude(m.z + u))
    }
}

// --- WGS84 transverse Mercator, Krueger's series to n^6 (Karney 2011, "Transverse Mercator with
// an accuracy of a few nanometers"): exact to well under a millimetre within 1000 km of the
// central meridian, which is what GeographicLib's TransverseMercator (Autoware's) computes too.

const A: f64 = 6_378_137.0;
const F: f64 = 1.0 / 298.257_223_563;

struct Series {
    a: f64,
    alpha: [f64; 6],
    beta: [f64; 6],
    delta: [f64; 6],
    e: f64,
}

fn series() -> Series {
    let n = F / (2.0 - F);
    let (n2, n3, n4, n5, n6) = (n * n, n.powi(3), n.powi(4), n.powi(5), n.powi(6));
    Series {
        a: A / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0),
        alpha: [
            n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0
                + 7891.0 * n6 / 37800.0,
            13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0
                - 1983433.0 * n6 / 1935360.0,
            61.0 * n3 / 240.0 - 103.0 * n4 / 140.0
                + 15061.0 * n5 / 26880.0
                + 167603.0 * n6 / 181440.0,
            49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0 + 6601661.0 * n6 / 7257600.0,
            34729.0 * n5 / 80640.0 - 3418889.0 * n6 / 1995840.0,
            212378941.0 * n6 / 319334400.0,
        ],
        beta: [
            n / 2.0 - 2.0 * n2 / 3.0 + 37.0 * n3 / 96.0 - n4 / 360.0 - 81.0 * n5 / 512.0
                + 96199.0 * n6 / 604800.0,
            n2 / 48.0 + n3 / 15.0 - 437.0 * n4 / 1440.0 + 46.0 * n5 / 105.0
                - 1118711.0 * n6 / 3870720.0,
            17.0 * n3 / 480.0 - 37.0 * n4 / 840.0 - 209.0 * n5 / 4480.0 + 5569.0 * n6 / 90720.0,
            4397.0 * n4 / 161280.0 - 11.0 * n5 / 504.0 - 830251.0 * n6 / 7257600.0,
            4583.0 * n5 / 161280.0 - 108847.0 * n6 / 3991680.0,
            20648693.0 * n6 / 638668800.0,
        ],
        delta: [
            2.0 * n - 2.0 * n2 / 3.0 - 2.0 * n3 + 116.0 * n4 / 45.0 + 26.0 * n5 / 45.0
                - 2854.0 * n6 / 675.0,
            7.0 * n2 / 3.0 - 8.0 * n3 / 5.0 - 227.0 * n4 / 45.0
                + 2704.0 * n5 / 315.0
                + 2323.0 * n6 / 945.0,
            56.0 * n3 / 15.0 - 136.0 * n4 / 35.0 - 1262.0 * n5 / 105.0 + 73814.0 * n6 / 2835.0,
            4279.0 * n4 / 630.0 - 332.0 * n5 / 35.0 - 399572.0 * n6 / 14175.0,
            4174.0 * n5 / 315.0 - 144838.0 * n6 / 6237.0,
            601676.0 * n6 / 22275.0,
        ],
        e: (F * (2.0 - F)).sqrt(),
    }
}

/// (easting, northing) of (lat, lon - lon0) in degrees, scale k0, no false origin.
fn tm_forward(lat: f64, dlon: f64, k0: f64) -> (f64, f64) {
    let s = series();
    let (phi, lam) = (lat.to_radians(), dlon.to_radians());
    let t = (phi.sin().atanh() - s.e * (s.e * phi.sin()).atanh()).sinh();
    let xi1 = t.atan2(lam.cos());
    let eta1 = (lam.sin() / (1.0 + t * t).sqrt()).atanh();
    let (mut xi, mut eta) = (xi1, eta1);
    for (j, a) in s.alpha.iter().enumerate() {
        let k = 2.0 * (j as f64 + 1.0);
        xi += a * (k * xi1).sin() * (k * eta1).cosh();
        eta += a * (k * xi1).cos() * (k * eta1).sinh();
    }
    (k0 * s.a * eta, k0 * s.a * xi)
}

/// (lat, lon) in degrees of (easting, northing) about central meridian lon0, scale k0.
fn tm_inverse(e: f64, n: f64, lon0: f64, k0: f64) -> (f64, f64) {
    let s = series();
    let (xi, eta) = (n / (k0 * s.a), e / (k0 * s.a));
    let (mut xi1, mut eta1) = (xi, eta);
    for (j, b) in s.beta.iter().enumerate() {
        let k = 2.0 * (j as f64 + 1.0);
        xi1 -= b * (k * xi).sin() * (k * eta).cosh();
        eta1 -= b * (k * xi).cos() * (k * eta).sinh();
    }
    let chi = (xi1.sin() / eta1.cosh()).asin();
    let mut phi = chi;
    for (j, d) in s.delta.iter().enumerate() {
        phi += d * (2.0 * (j as f64 + 1.0) * chi).sin();
    }
    let lam = eta1.sinh().atan2(xi1.cos());
    (phi.to_degrees(), lon0 + lam.to_degrees())
}

/// An MGRS 100 km square (`<zone><band><column><row>`, e.g. `51RUH`): its UTM zone, hemisphere
/// and south-west corner (UTM easting, northing with the southern hemisphere's 10 000 km).
fn mgrs_square(grid: &str) -> Result<MapProjector, String> {
    let bad = || format!("map_projector_info: mgrs_grid {grid}: not a 100 km square like 51RUH");
    let g = grid.trim().to_ascii_uppercase();
    let digits: String = g.chars().take_while(|c| c.is_ascii_digit()).collect();
    let zone: u32 = digits.parse().map_err(|_| bad())?;
    let letters: Vec<char> = g[digits.len()..].chars().collect();
    if !(1..=60).contains(&zone) || letters.len() != 3 {
        return Err(bad());
    }
    const BANDS: &str = "CDEFGHJKLMNPQRSTUVWX";
    let band = BANDS.find(letters[0]).ok_or_else(bad)?;
    let north = letters[0] >= 'N';
    // columns: sets of 8 letters (A-H, J-R, S-Z), by zone mod 3
    const COLS: [&str; 3] = ["ABCDEFGH", "JKLMNPQR", "STUVWXYZ"];
    let col = COLS[((zone - 1) % 3) as usize]
        .find(letters[1])
        .ok_or_else(bad)?;
    let e0 = (col as f64 + 1.0) * 100_000.0;
    // rows: 20 letters A-V without I and O, cycling every 2000 km, even zones offset by 5
    const ROWS: &str = "ABCDEFGHJKLMNPQRSTUV";
    let row = ROWS.find(letters[2]).ok_or_else(bad)?;
    let row = (row + 20 - if zone.is_multiple_of(2) { 5 } else { 0 }) % 20;
    let n_mod = row as f64 * 100_000.0;
    // the band's lowest northing settles which 2000 km cycle: its south edge, at this zone's
    // central meridian (a band spans 8 degrees, about 885 km, under a cycle)
    let band_lat = -80.0 + 8.0 * band as f64;
    let (_, band_n) = tm_forward(band_lat, 0.0, 0.9996);
    let band_n = if north { band_n } else { band_n + 10_000_000.0 };
    let mut n0 = n_mod;
    while n0 + 100_000.0 <= band_n - 100_000.0 {
        n0 += 2_000_000.0;
    }
    Ok(MapProjector::Mgrs {
        zone,
        north,
        e0,
        n0,
    })
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
        assert_eq!(
            p,
            MapProjector::TransverseMercator {
                lat0: 0.0,
                lon0: 0.0,
                alt0: 0.0,
                k0: 1.0
            }
        );
    }

    #[test]
    fn a_file_without_scale_factor_takes_the_loaders_default() {
        let text =
            "# a course fixture\nprojector_type: TransverseMercator\nvertical_datum: WGS84\n\
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
        let p = MapProjector::from_yaml(
            "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 51RUH\n",
        )
        .unwrap();
        assert_eq!(
            p,
            MapProjector::Mgrs {
                zone: 51,
                north: true,
                e0: 300_000.0,
                n0: 2_700_000.0
            }
        );
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
        let p = MapProjector::TransverseMercator {
            lat0: 0.0,
            lon0: 0.0,
            alt0: 0.0,
            k0: 1.0,
        };
        let (lat, lon) = p.lat_lon(190.8, -130.1);
        // pyproj 3.3.0: +proj=tmerc +lat_0=0 +lon_0=0 +k=1 +ellps=WGS84, inverse of (190.8, -130.1)
        assert!(
            metres((lat, lon), (-0.0011765846891, 0.0017139855622)) < 0.01,
            "{lat} {lon}"
        );
        // CARLA's own lat/lon for the same place is 0.82 m from it
        assert!(metres((lat, lon), (-0.001169175909, 0.001714671458)) > 0.8);
    }

    fn town01() -> GnssGeo {
        GnssGeo {
            projector: CARLA_DEFAULT,
            map_origin: Default::default(),
        }
    }

    /// A fix is the map point of the sensor's CARLA location (y mirrored), through the projector
    #[test]
    fn a_fix_is_the_sensors_map_point_through_the_projector() {
        let g = town01();
        let (lat, lon, alt) = g.fix([190.8, 130.1, 2.1], [0.0, 0.0, 0.0]);
        assert!(
            metres((lat, lon), (-0.0011765846891, 0.0017139855622)) < 0.001,
            "{lat} {lon}"
        );
        assert!((alt - 2.1).abs() < 1e-9);
    }

    #[test]
    fn the_receivers_error_moves_the_fix_in_the_maps_metres() {
        let g = town01();
        let (lat, lon, alt) = g.fix([190.8, 130.1, 2.1], [1.0, -2.0, 0.5]);
        let want = CARLA_DEFAULT.lat_lon(191.8, -132.1);
        assert!(metres((lat, lon), want) < 0.001, "{lat} {lon} {want:?}");
        assert!((alt - 2.6).abs() < 1e-9);
    }

    #[test]
    fn the_map_origin_offsets_the_fix() {
        let g = GnssGeo {
            projector: CARLA_DEFAULT,
            map_origin: crate::coordinate_conversion::MapOrigin {
                x: 10.0,
                y: -5.0,
                z: 1.0,
            },
        };
        let (lat, lon, alt) = g.fix([190.8, 130.1, 2.1], [0.0, 0.0, 0.0]);
        assert!(metres((lat, lon), CARLA_DEFAULT.lat_lon(200.8, -135.1)) < 0.001);
        assert!((alt - 3.1).abs() < 1e-9);
    }

    #[test]
    fn an_empty_path_is_carlas_convention_and_a_missing_file_an_error() {
        assert_eq!(
            GnssGeo::load("", Default::default()).unwrap().projector,
            CARLA_DEFAULT
        );
        assert_eq!(
            GnssGeo::load("none", Default::default()).unwrap().projector,
            CARLA_DEFAULT
        );
        let e =
            GnssGeo::load("/nonexistent/map_projector_info.yaml", Default::default()).unwrap_err();
        assert!(e.contains("/nonexistent/map_projector_info.yaml"), "{e}");
    }
}
