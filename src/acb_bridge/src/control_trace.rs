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
//! 20 Hz, formatted on the caller and written by a thread of its own ([`NonBlockingLog`]): the
//! rows used to be written and flushed from the control callback and the main loop, which is
//! the log-disk stall `log_writer` exists to keep off those paths (scensim issue 135). A row
//! the writer had to drop (a stalled disk) is counted in the file, never waited for.

use std::{
    fs::File,
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use crate::{
    error::Result,
    log_writer::{LogGuard, NonBlockingLog, QUEUE_LINES},
};

/// Timestamps for one command's passage through the bridge.
pub struct ControlTrace {
    out: NonBlockingLog,
    /// Waits, bounded, for queued rows at drop.
    _guard: LogGuard,
    /// Commands applied since the last `loop` row. The `pumped` column used to be the
    /// literal 1, which reads in the CSV exactly like a measurement of "one callback per
    /// iteration" and is not one -- it cannot disagree with the assumption it was meant to
    /// check. This counts them.
    applied_since_loop: AtomicU64,
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
        let (out, guard) = NonBlockingLog::new(file, QUEUE_LINES);
        out.line(
            "# per-stage control latency, written by acb_bridge\n\
             # stamp_s: the command's own header stamp (simulation time)\n\
             # recv_s: monotonic time the subscription callback began\n\
             # convert_us: turning the command into a CARLA control\n\
             # apply_us: the apply_control RPC itself\n\
             # age_ms: node ROS time at callback entry minus the command's stamp -- both are\n\
             #   simulation time, so this is the command's true staleness. An earlier\n\
             #   stamp-against-wall-clock figure removed a constant offset and so measured\n\
             #   jitter, not delay: any standing queue was subtracted out with the clock\n\
             #   difference.\n\
             #\n\
             # A `loop` row is one iteration of the bridge's main loop rather than a command:\n\
             # period_us is since the previous iteration, spin_us is the executor pump, and\n\
             # pumped is how many control commands were APPLIED during it -- counted, not\n\
             #   assumed. It was previously the literal 1.\n\
             kind,stamp_s,recv_s,convert_us,apply_us,age_ms",
        );
        Ok(Self {
            out,
            _guard: guard,
            applied_since_loop: AtomicU64::new(0),
            origin: Instant::now(),
        })
    }

    /// One iteration of the main loop: how long since the last, how long the executor pump
    /// took, and how many callbacks it actually ran.
    ///
    /// The pump matters because it is `spin_once`, which runs at most one callback per
    /// iteration. With several subscriptions live, a control command can wait for as many
    /// iterations as there are other callbacks queued ahead of it, and that wait is invisible
    /// to a trace that starts when the callback begins.
    pub fn record_loop(&self, period: std::time::Duration, spin: std::time::Duration) {
        let applied = self.applied_since_loop.swap(0, Ordering::Relaxed);
        let row = format!(
            "loop,0,0,{:.1},{:.1},{}",
            period.as_micros() as f64,
            spin.as_micros() as f64,
            applied
        );
        // The writer flushes each line: a buffered trace that only reaches disk when something
        // else happens reads as an empty file, which is how the first loop-only capture looked.
        self.out.line(&row);
    }

    pub fn record(
        &self,
        stamp_s: f64,
        recv: Instant,
        converted: Instant,
        applied: Instant,
        age_ms: f64,
    ) {
        let row = format!(
            "cmd,{:.6},{:.6},{:.1},{:.1},{:.2}",
            stamp_s,
            recv.duration_since(self.origin).as_secs_f64(),
            converted.duration_since(recv).as_micros() as f64,
            applied.duration_since(converted).as_micros() as f64,
            age_ms,
        );
        self.applied_since_loop.fetch_add(1, Ordering::Relaxed);
        // A failed or stalled trace write never takes the run down with it, nor waits.
        self.out.line(&row);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rows reach the file through the writer thread (scensim issue 135), header first,
    /// each one line, all of them by the time the trace is dropped.
    #[test]
    fn rows_reach_the_file_through_the_writer() {
        let path = std::env::temp_dir().join(format!(
            "acb-control-trace-{}-{:?}.csv",
            std::process::id(),
            std::thread::current().id()
        ));
        {
            let t = ControlTrace::create(&path).unwrap();
            let t0 = Instant::now();
            t.record(1000.5, t0, t0, t0, 12.5);
            t.record_loop(
                std::time::Duration::from_micros(50_000),
                std::time::Duration::from_micros(800),
            );
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.starts_with("# per-stage control latency"), "{text}");
        let rows: Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
        assert_eq!(rows[0], "kind,stamp_s,recv_s,convert_us,apply_us,age_ms");
        assert!(rows[1].starts_with("cmd,1000.500000,"), "{text}");
        assert!(rows[1].ends_with(",12.50"), "{text}");
        assert_eq!(rows[2], "loop,0,0,50000.0,800.0,1");
        assert_eq!(rows.len(), 3, "{text}");
    }
}
