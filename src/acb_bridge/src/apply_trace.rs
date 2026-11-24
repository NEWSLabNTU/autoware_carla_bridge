//! When each actuation of the hero left this bridge, against CARLA's frames (scensim issue 135).
//!
//! LibCarla's `apply_control` and `set_target_velocity` are asynchronous calls
//! (`AsyncCall` in `carla/client/detail/Client.cpp`), and this bridge is not the client that
//! ticks the world: the world server is. So a control sent for tick k can reach the server
//! after tick k began, and the hero runs that tick on its previous control. The world server
//! now records the control CARLA held at every frame (the truth's `control`); this records
//! what was sent, and when, so the two can be joined by frame.
//!
//! One CSV row per actuation call and one per frame the main loop takes:
//!
//! - `apply`: `mono_ns` (`CLOCK_MONOTONIC` just before the call, comparable across the
//!   host's processes), `f_obs` (the newest frame the main loop had taken), `f_after` (the
//!   newest frame this client had received when the call returned: `World::snapshot`, the
//!   client's own copy, no RPC; above `f_obs`, a tick finished during the call), the control
//!   as sent, and `call_us`;
//! - `velocity`: the same for a `set_target_velocity` (the physics keeper's), no control;
//! - `tick`: a frame the main loop took, and when.
//!
//! A `# boot_id <id>` line before the header names this host's boot
//! (`/proc/sys/kernel/random/boot_id`): `mono_ns` compares with the world server's tick stamps
//! only under the same one (scensim issue 135 step 3).
//!
//! Off unless `apply_trace_path` is set. Rows are formatted on the caller and written by a
//! thread of their own ([`NonBlockingLog`]), never on the control path's disk; a row dropped
//! while the disk stalls is counted in the file as `[acb_bridge log] N line(s) dropped ...`.

use std::{
    path::Path,
    sync::{
        atomic::{AtomicU64, Ordering},
        Mutex, OnceLock,
    },
};

use carla::{client::World, rpc::VehicleControl};

use crate::{
    error::Result,
    log_writer::{LogGuard, NonBlockingLog, QUEUE_LINES},
};

pub const HEADER: &str = "kind,mono_ns,f_obs,f_after,throttle,brake,steer,hand_brake,reverse,\
                          gear,call_us";

struct ApplyTrace {
    out: NonBlockingLog,
    _guard: LogGuard,
    /// The session's world (a respawn or a map load starts a new one: [`set_world`]).
    world: Mutex<Option<World>>,
}

static TRACE: OnceLock<ApplyTrace> = OnceLock::new();
/// The newest frame the main loop has taken.
static OBSERVED: AtomicU64 = AtomicU64::new(0);

/// Start the trace at `path` (once per process; a second call is ignored).
pub fn install(path: &Path) -> Result<()> {
    let file = std::fs::File::create(path).map_err(|e| {
        crate::error::BridgeError::ConfigError(format!(
            "cannot create the apply trace at {}: {e}",
            path.display()
        ))
    })?;
    let (out, guard) = NonBlockingLog::new(file, QUEUE_LINES);
    out.line(
        "# when each actuation of the hero left acb_bridge, against CARLA's frames \
         (scensim issue 135; apply_trace.rs)",
    );
    out.line(&boot_id_line(&boot_id()));
    out.line(HEADER);
    let _ = TRACE.set(ApplyTrace {
        out,
        _guard: guard,
        world: Mutex::new(None),
    });
    Ok(())
}

/// The world this session's frames come from, for `f_after`.
pub fn set_world(world: &World) {
    if let Some(t) = TRACE.get() {
        if let Ok(mut w) = t.world.lock() {
            *w = Some(world.clone());
        }
    }
}

/// `CLOCK_MONOTONIC`, ns: the host's one clock every process reads alike.
pub fn mono_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: a valid pointer to a timespec; CLOCK_MONOTONIC always exists on Linux.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec * 1_000_000_000 + ts.tv_nsec
}

/// The main loop took `frame` (call once per frame, before the executor runs the commands
/// that frame's spin drains).
pub fn observe(frame: u64) {
    OBSERVED.store(frame, Ordering::Relaxed);
    if let Some(t) = TRACE.get() {
        t.out.line(&tick_row(mono_ns(), frame));
    }
}

/// An actuation call that began at `before_ns` (from [`mono_ns`]) has returned: `control`
/// for an `apply_control`, None for a `set_target_velocity`.
pub fn applied(before_ns: i64, control: Option<&VehicleControl>) {
    let Some(t) = TRACE.get() else {
        return;
    };
    let after_ns = mono_ns();
    let f_after = t
        .world
        .lock()
        .ok()
        .and_then(|w| w.as_ref().and_then(|w| w.snapshot().ok()))
        .map(|s| s.frame() as u64);
    t.out.line(&apply_row(
        before_ns,
        OBSERVED.load(Ordering::Relaxed),
        f_after,
        control,
        (after_ns - before_ns) as f64 / 1e3,
    ));
}

/// This host's boot id, empty where it cannot be read.
fn boot_id() -> String {
    std::fs::read_to_string("/proc/sys/kernel/random/boot_id")
        .map(|s| s.trim().to_string())
        .unwrap_or_default()
}

fn boot_id_line(id: &str) -> String {
    format!("# boot_id {id}")
}

fn tick_row(mono_ns: i64, frame: u64) -> String {
    format!("tick,{mono_ns},{frame},,,,,,,,")
}

fn apply_row(
    mono_ns: i64,
    f_obs: u64,
    f_after: Option<u64>,
    control: Option<&VehicleControl>,
    call_us: f64,
) -> String {
    let after = f_after.map(|f| f.to_string()).unwrap_or_default();
    match control {
        // CARLA keeps the control as float32: printed in full so a reader can match it to the
        // truth's `control` within float32's resolution
        Some(c) => format!(
            "apply,{mono_ns},{f_obs},{after},{:?},{:?},{:?},{},{},{},{call_us:.1}",
            c.throttle,
            c.brake,
            c.steer,
            u8::from(c.hand_brake),
            u8::from(c.reverse),
            c.gear
        ),
        None => format!("velocity,{mono_ns},{f_obs},{after},,,,,,,{call_us:.1}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn control() -> VehicleControl {
        VehicleControl {
            throttle: 0.3,
            steer: -0.125,
            brake: 0.0,
            hand_brake: false,
            reverse: true,
            manual_gear_shift: false,
            gear: -1,
        }
    }

    #[test]
    fn every_row_has_the_headers_columns() {
        let n = HEADER.split(',').count();
        for row in [
            tick_row(5, 42),
            apply_row(7, 42, Some(43), Some(&control()), 12.34),
            apply_row(7, 42, None, None, 3.0),
        ] {
            assert_eq!(row.split(',').count(), n, "{row}");
        }
    }

    #[test]
    fn an_apply_row_carries_the_control_as_sent() {
        assert_eq!(
            apply_row(7, 42, Some(43), Some(&control()), 12.34),
            "apply,7,42,43,0.3,0.0,-0.125,0,1,-1,12.3"
        );
        assert_eq!(
            apply_row(9, 42, None, None, 3.0),
            "velocity,9,42,,,,,,,,3.0"
        );
        assert_eq!(tick_row(5, 42), "tick,5,42,,,,,,,,");
    }

    #[test]
    fn the_boot_id_line_names_this_hosts_boot() {
        let id = boot_id();
        assert_eq!(id.len(), 36, "{id}");
        assert_eq!(boot_id_line(&id), format!("# boot_id {id}"));
    }

    #[test]
    fn the_clock_is_monotonic_and_in_ns() {
        let a = mono_ns();
        std::thread::sleep(std::time::Duration::from_millis(2));
        let b = mono_ns();
        assert!(b - a >= 2_000_000 && b - a < 2_000_000_000, "{a} {b}");
    }
}
