//! Is a CARLA client's RPC connection still alive? (the port-2000 hang, 2026-10-16)
//!
//! A CARLA 0.9.16 client outlives its server badly. Its RPC connection never reconnects:
//! once the server is gone every call times out, forever. LibCarla's streaming client does
//! reconnect, to whatever server comes up next on the same port, and the world-tick stream's
//! token is the same on every server, so an `on_tick` subscription silently resumes on the
//! new server. Frames keep arriving, which is why the stall checks below never fired: they
//! only asked when frames stopped. On a frame flagged with a light update (any map load) the
//! client's LightManager queries the lights from inside the stream callback over the dead
//! RPC connection and blocks for the client's whole timeout without reading the stream, and
//! a synchronous server busy-waits on its game thread until every subscriber took the last
//! frame (`ServerSession::Write`). The whole server then stops answering for that timeout,
//! once per stale client and light update.
//!
//! So each long-lived client asks every [`PROBE_EVERY`], whether frames arrive or not, for
//! the server version. CARLA answers that on its RPC threads, without the game thread, so a
//! live server answers in milliseconds even while its world waits for a tick; a dead
//! connection fails after [`PROBE_TIMEOUT`]. After [`PROBE_FAILURES`] failures in a row the
//! caller drops the client (and every world and actor handle from it), which ends its
//! stream subscriptions, and connects afresh. The probe leaves a dead client with a 1 ms
//! timeout first, so the drop does not itself wait on the dead connection. Two failures
//! take 2-4 s; a restarted CARLA serves its first frames no sooner than ~8 s after the old
//! one died, so the stale client is gone before its subscription can reattach.

use std::time::{Duration, Instant};

use carla::client::Client;

/// How often a client is probed.
pub const PROBE_EVERY: Duration = Duration::from_secs(1);
/// The RPC timeout of one probe.
pub const PROBE_TIMEOUT: Duration = Duration::from_secs(2);
/// Consecutive failed probes before the connection counts as dead.
pub const PROBE_FAILURES: u32 = 2;
/// The RPC timeout a dead client is left with. LibCarla's destructors make RPCs (the
/// LightManager pushes its pending light changes as it goes), and on a dead connection each
/// would wait the client's whole timeout while the stream subscription is still alive.
const DEAD_TIMEOUT: Duration = Duration::from_millis(1);

/// Probes one client; see the module documentation.
pub struct ConnectionProbe {
    last: Instant,
    failures: u32,
    /// The client's own RPC timeout, restored after each probe.
    normal_timeout: Duration,
}

impl ConnectionProbe {
    pub fn new(normal_timeout: Duration) -> Self {
        Self {
            last: Instant::now(),
            failures: 0,
            normal_timeout,
        }
    }

    /// Whether the connection is dead. Cheap to call often: it probes at most every
    /// [`PROBE_EVERY`] and otherwise returns `false` at once.
    pub fn dead(&mut self, client: &mut Client) -> bool {
        if self.last.elapsed() < PROBE_EVERY {
            return false;
        }
        let _ = client.set_timeout(PROBE_TIMEOUT);
        let answered = client.server_version();
        let _ = client.set_timeout(self.normal_timeout);
        self.last = Instant::now();
        let dead = self.record(answered.map(|_| ()).map_err(|e| e.to_string()));
        if dead {
            let _ = client.set_timeout(DEAD_TIMEOUT);
        }
        dead
    }

    fn record(&mut self, answered: Result<(), String>) -> bool {
        match answered {
            Ok(()) => {
                self.failures = 0;
                false
            }
            Err(e) => {
                self.failures += 1;
                tracing::warn!(
                    "CARLA did not answer the version probe ({e}); {} of {PROBE_FAILURES}",
                    self.failures
                );
                self.failures >= PROBE_FAILURES
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dead_after_consecutive_failures_only() {
        let mut p = ConnectionProbe::new(Duration::from_secs(10));
        assert!(!p.record(Err("timeout".into())));
        assert!(!p.record(Ok(())), "a success resets the count");
        assert!(!p.record(Err("timeout".into())));
        assert!(p.record(Err("timeout".into())));
    }
}
