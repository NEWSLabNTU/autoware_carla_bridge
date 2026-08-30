//! Per-stage timing for one control command, written as it happens.
//!
//! Every figure this project has had for the command-to-wheel delay -- 478 ms, then 200 ms on
//! re-measurement, then a second from a cross-correlation -- was inferred from the steering
//! command during a scenario. That signal has no edges: its median change is 0.00028 rad per
//! message across a total excursion of 0.067, and a run contains two steps worth timing. No
//! estimator can recover a delay from it, which is why the three attempts disagreed by a factor
//! of five (docs/issues/016).
//!
//! So this records the stages instead of inferring them. Each command that reaches the bridge
//! writes one row: when Autoware stamped it, when the callback saw it, how long the conversion
//! took, and how long CARLA's `apply_control` RPC took. Nothing here is a correlation, and the
//! only unmeasured segment left is inside the simulator.
//!
//! Off unless `control_trace_path` is set. The cost is one line of CSV per control message at
//! 20 Hz, and a lock that is otherwise never contended.

use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    sync::Mutex,
    time::Instant,
};

use crate::error::Result;

/// Timestamps for one command's passage through the bridge.
pub struct ControlTrace {
    out: Mutex<BufWriter<File>>,
    /// Wall clock is not comparable to a simulation-time stamp, so durations are taken from a
    /// monotonic origin and the stamp is recorded as-is for the caller to align.
    origin: Instant,
}

impl ControlTrace {
    pub fn create(path: &Path) -> Result<Self> {
        // Mapped rather than propagated: BridgeError has no From<io::Error>, and a trace file
        // is a diagnostic, so its failure should name itself rather than borrow another kind.
        let file = File::create(path).map_err(|e| {
            crate::error::BridgeError::ConfigError(format!(
                "cannot create the control trace at {}: {e}",
                path.display()
            ))
        })?;
        let mut out = BufWriter::new(file);
        let header = writeln!(
            out,
            "# per-stage control latency, written by acb_bridge\n\
             # stamp_s: the command's own header stamp (simulation time)\n\
             # recv_s: monotonic time the subscription callback began\n\
             # convert_us: turning the command into a CARLA control\n\
             # apply_us: the apply_control RPC itself\n\
             stamp_s,recv_s,convert_us,apply_us"
        )
        .and_then(|()| out.flush());
        header.map_err(|e| {
            crate::error::BridgeError::ConfigError(format!(
                "cannot write the control trace header at {}: {e}",
                path.display()
            ))
        })?;
        Ok(Self {
            out: Mutex::new(out),
            origin: Instant::now(),
        })
    }

    pub fn now(&self) -> Instant {
        Instant::now()
    }

    pub fn record(&self, stamp_s: f64, recv: Instant, converted: Instant, applied: Instant) {
        let row = format!(
            "{:.6},{:.6},{:.1},{:.1}",
            stamp_s,
            recv.duration_since(self.origin).as_secs_f64(),
            converted.duration_since(recv).as_micros() as f64,
            applied.duration_since(converted).as_micros() as f64,
        );
        if let Ok(mut out) = self.out.lock() {
            // A failed trace write must never take the run down with it.
            let _ = writeln!(out, "{row}");
            let _ = out.flush();
        }
    }
}
