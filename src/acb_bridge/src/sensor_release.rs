//! Release our sensors on request, before someone else destroys the vehicle.
//!
//! # Why this exists
//!
//! This bridge attaches sensors to a vehicle it does not own and calls `Listen()` on
//! them, so it owns their stream sessions. The scenario runner owns the vehicle's
//! lifetime and despawns it with `destroy_with_children`, which destroys those sensors
//! underneath us.
//!
//! After that there is no way to unsubscribe: `Sensor::stop()` on a destroyed actor fails
//! with `close: Bad file descriptor`, and the client then retries the dead stream forever.
//! CARLA 0.9.16 answers those retries at ~48,000 `Invalid session: no stream available
//! with id N` per second -- 2.6 MB/s of log -- until the server segfaults. See
//! `docs/issues/015`.
//!
//! Stopping *before* the destroy costs nothing and leaves nothing behind. The only thing
//! missing was for the two processes to agree on when, which is what this channel is.
//!
//! # Protocol
//!
//! Deliberately tiny and human-readable, so `zmqcat` is enough to debug it.
//!
//! | direction | socket | frame |
//! |---|---|---|
//! | csb -> acb | PUB / SUB | `release <actor_id> <role_name>` |
//! | acb -> csb | PULL / PUSH | `released <actor_id>` |
//!
//! We match on `actor_id`: it is the one key both sides always have. The scenario
//! runner's ledger teardown path knows the id but not the role name, so the name is
//! carried for logging only.
//!
//! Best effort by design. If this bridge is not running, or is slow, the scenario runner
//! waits out a short timeout and despawns anyway -- a noisy teardown is better than a
//! stalled scenario.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::JoinHandle,
    time::Duration,
};

use crate::carla_vehicle::CarlaVehicle;

/// How long to block in `recv` before re-checking the shutdown flag.
const POLL_TIMEOUT: Duration = Duration::from_millis(200);

/// Listens for release requests and stops our sensors when one names our vehicle.
///
/// Dropping this signals the thread and joins it.
pub struct SensorReleaseListener {
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl SensorReleaseListener {
    /// Connect to the scenario runner's release channel and start listening.
    ///
    /// `notify_endpoint` is its PUB socket, `ack_endpoint` its PULL socket. Returns `None`
    /// if either socket cannot be set up -- this is an optimisation, not a requirement, so
    /// a bridge that cannot reach the channel still runs.
    pub fn start(
        notify_endpoint: &str,
        ack_endpoint: &str,
        vehicle: Arc<Mutex<CarlaVehicle>>,
        vehicle_id: u32,
    ) -> Option<Self> {
        let ctx = zmq::Context::new();

        let sub = match ctx.socket(zmq::SUB) {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!("Sensor release channel: cannot create SUB socket: {e}");
                return None;
            }
        };
        if let Err(e) = sub.connect(notify_endpoint) {
            tracing::warn!("Sensor release channel: cannot connect to {notify_endpoint}: {e}");
            return None;
        }
        if let Err(e) = sub.set_subscribe(b"") {
            tracing::warn!("Sensor release channel: cannot subscribe: {e}");
            return None;
        }
        if let Err(e) = sub.set_rcvtimeo(POLL_TIMEOUT.as_millis() as i32) {
            tracing::warn!("Sensor release channel: cannot set receive timeout: {e}");
            return None;
        }

        let push = match ctx.socket(zmq::PUSH) {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!("Sensor release channel: cannot create PUSH socket: {e}");
                return None;
            }
        };
        // Never block the listener on a scenario runner that has gone away, and never let
        // acks pile up: an unsent ack is stale the moment its deadline passes.
        let _ = push.set_sndtimeo(POLL_TIMEOUT.as_millis() as i32);
        let _ = push.set_linger(0);
        let _ = push.set_sndhwm(16);
        if let Err(e) = push.connect(ack_endpoint) {
            tracing::warn!("Sensor release channel: cannot connect to {ack_endpoint}: {e}");
            return None;
        }

        tracing::info!(
            "Sensor release channel: listening on {notify_endpoint}, acking to \
             {ack_endpoint}, for actor {vehicle_id}"
        );

        let running = Arc::new(AtomicBool::new(true));
        let thread_running = running.clone();
        let handle = std::thread::Builder::new()
            .name("sensor-release".into())
            .spawn(move || {
                while thread_running.load(Ordering::SeqCst) {
                    let msg = match sub.recv_string(0) {
                        Ok(Ok(s)) => s,
                        // Timeout, or a frame that was not UTF-8. Neither is fatal.
                        Ok(Err(_)) | Err(_) => continue,
                    };

                    let Some(request) = ReleaseRequest::parse(&msg) else {
                        tracing::debug!("Sensor release channel: unparseable frame {msg:?}");
                        continue;
                    };
                    if request.actor_id != vehicle_id {
                        continue;
                    }

                    tracing::info!(
                        "Release requested for '{}' (actor {}): stopping our sensors before \
                         the scenario runner destroys them",
                        request.role_name,
                        request.actor_id
                    );

                    // Stop only. The requester destroys the actors a moment later, and a
                    // stopped sensor leaves no session behind for it to orphan.
                    match vehicle.lock() {
                        Ok(mut vehicle) => vehicle.stop_sensors(),
                        Err(e) => {
                            tracing::warn!("Sensor release channel: vehicle lock poisoned: {e}");
                        }
                    }

                    if let Err(e) = push.send(format!("released {}", request.actor_id).as_str(), 0)
                    {
                        tracing::debug!("Sensor release channel: could not ack: {e}");
                    }
                }
            })
            .ok()?;

        Some(Self {
            running,
            handle: Some(handle),
        })
    }
}

impl Drop for SensorReleaseListener {
    fn drop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// A parsed `release <actor_id> <role_name>` frame.
#[derive(Debug, PartialEq, Eq)]
struct ReleaseRequest {
    actor_id: u32,
    role_name: String,
}

impl ReleaseRequest {
    fn parse(frame: &str) -> Option<Self> {
        let mut parts = frame.split_whitespace();
        if parts.next()? != "release" {
            return None;
        }
        let actor_id = parts.next()?.parse().ok()?;
        // The ledger teardown path knows the id but not the name, and sends "-".
        let role_name = parts.next().unwrap_or("-").to_string();
        Some(Self {
            actor_id,
            role_name,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_release_frame() {
        let r = ReleaseRequest::parse("release 187 hero").unwrap();
        assert_eq!(r.actor_id, 187);
        assert_eq!(r.role_name, "hero");
    }

    /// The ledger teardown path has no role name for the actor it is destroying.
    #[test]
    fn a_missing_role_name_is_allowed() {
        let r = ReleaseRequest::parse("release 42").unwrap();
        assert_eq!(r.actor_id, 42);
        assert_eq!(r.role_name, "-");
    }

    #[test]
    fn rejects_other_verbs_and_junk() {
        assert!(ReleaseRequest::parse("released 187").is_none());
        assert!(ReleaseRequest::parse("release notanid").is_none());
        assert!(ReleaseRequest::parse("").is_none());
        assert!(ReleaseRequest::parse("release").is_none());
    }

    /// Trailing fields are ignored rather than rejected, so the frame can grow.
    #[test]
    fn extra_fields_do_not_break_parsing() {
        let r = ReleaseRequest::parse("release 7 bg_av_1 something_new").unwrap();
        assert_eq!(r.actor_id, 7);
        assert_eq!(r.role_name, "bg_av_1");
    }
}
