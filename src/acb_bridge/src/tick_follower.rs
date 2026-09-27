//! Follow CARLA's frames without losing the ones that finish while the bridge is busy.
//!
//! The main loop used to wait with `World::wait_for_tick_or_timeout`. That call waits for
//! the *next* tick after it is made, so a frame that completes while the loop is publishing
//! is never seen: the loop comes back, waits again, and gets the frame after it. Sensor data
//! and the frame it belongs to are perishable, so each missed frame is lost outright, and
//! the loop spends its idle time blocked on a frame it could already have been handling.
//! autoware_universe's `tick_follower` (PR #13286) documents the same failure.
//!
//! The fix is theirs too. Register `World::on_tick` once; CARLA calls it on its own thread
//! for every frame, and it only pushes the snapshot onto a bounded queue. The main loop
//! drains that queue to the NEWEST snapshot -- only the latest state means anything to a
//! publisher -- counts what it drained over, and does its per-frame work once.
//!
//! This never ticks the world. Only the simulation bridge ticks (the single-ticker
//! invariant, `docs/design/multi-instance-architecture.md`); `on_tick` and
//! `wait_for_tick_or_timeout` both observe without driving.
//!
//! A callback that stops firing says nothing about *why* -- a paused simulation and a dead
//! server look the same from inside the queue. So when the queue stays empty the follower
//! still asks the server, with a short `wait_for_tick_or_timeout`, and returns its error
//! unchanged. The main loop classifies that exactly as before (roadmap 011: a timeout is
//! Idle, anything else is ConnectionLost).

use std::{
    collections::VecDeque,
    sync::{Arc, Condvar, Mutex},
    time::{Duration, Instant},
};

use carla::client::{OnTickId, World, WorldSnapshot};

/// Snapshots held while the main loop is busy. At a 100 ms server step this is almost a
/// second of backlog, far more than one slow iteration; anything older is dropped and
/// counted as skipped.
const QUEUE_CAPACITY: usize = 8;

/// How long the liveness probe waits when the queue is empty. It only has to see an error;
/// the frame, if one arrives, comes through the callback anyway.
const PROBE_TIMEOUT: Duration = Duration::from_millis(1);

/// Something with a frame number the drain logic can order by. Implemented for the real
/// snapshot and for the tests' mock.
pub trait Framed {
    fn frame_number(&self) -> usize;
}

impl Framed for WorldSnapshot {
    fn frame_number(&self) -> usize {
        self.frame()
    }
}

/// A frame this far *behind* the last one delivered is a new episode (a world reload
/// restarts the counter), not a late duplicate.
const EPISODE_RESET_GAP: usize = 1000;

/// Whether `frame` has already been delivered, or is older than what has.
fn is_stale(frame: usize, last_delivered: Option<usize>) -> bool {
    match last_delivered {
        Some(last) => frame <= last && last - frame < EPISODE_RESET_GAP,
        None => false,
    }
}

/// What one drain produced.
#[derive(Debug)]
pub struct Drained<T> {
    /// The newest frame not yet delivered, if any.
    pub newest: Option<T>,
    /// Frames that arrived and were passed over for a newer one, including any the full
    /// queue had to drop.
    pub skipped: u64,
}

struct QueueState<T> {
    items: VecDeque<T>,
    /// Pushed out of a full queue since the last drain.
    dropped: u64,
}

/// A bounded queue of frames, filled by the CARLA callback and drained by the main loop.
pub struct TickQueue<T> {
    state: Mutex<QueueState<T>>,
    ready: Condvar,
    capacity: usize,
}

impl<T: Framed> TickQueue<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            state: Mutex::new(QueueState {
                items: VecDeque::with_capacity(capacity),
                dropped: 0,
            }),
            ready: Condvar::new(),
            capacity: capacity.max(1),
        }
    }

    /// Called from CARLA's callback thread. Must stay cheap: no I/O, no logging.
    pub fn push(&self, item: T) {
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if state.items.len() >= self.capacity {
            state.items.pop_front();
            state.dropped += 1;
        }
        state.items.push_back(item);
        drop(state);
        self.ready.notify_one();
    }

    /// Take everything queued, keep the newest frame after `last_delivered`, and count the
    /// rest. Waits up to `timeout` for something to arrive if the queue is empty.
    pub fn drain_newest(&self, timeout: Duration, last_delivered: Option<usize>) -> Drained<T> {
        let state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        let mut state = if state.items.is_empty() && !timeout.is_zero() {
            self.ready
                .wait_timeout_while(state, timeout, |s| s.items.is_empty())
                .unwrap_or_else(|e| e.into_inner())
                .0
        } else {
            state
        };

        let mut skipped = std::mem::take(&mut state.dropped);
        let mut newest = None;
        for item in state.items.drain(..) {
            if is_stale(item.frame_number(), last_delivered) {
                // Already delivered (the liveness probe can see a frame before its
                // callback does), or older than what was. Not a skip.
                continue;
            }
            if newest.replace(item).is_some() {
                skipped += 1;
            }
        }
        Drained { newest, skipped }
    }
}

/// Follows one world's frames for one vehicle session. Unregisters its callback on drop.
pub struct TickFollower {
    world: World,
    callback: Option<OnTickId>,
    queue: Arc<TickQueue<WorldSnapshot>>,
    last_frame: Option<usize>,
    frames_processed: u64,
    frames_skipped: u64,
    /// Skips not yet reported, and when the last report went out. A loop that is steadily
    /// slower than the server skips on every iteration; one line per iteration would bury
    /// everything else, so the warning is batched.
    unreported_skips: u64,
    last_skip_warning: Option<Instant>,
}

/// At most one skipped-frame warning per this interval, carrying the count since the last.
const SKIP_WARNING_INTERVAL: Duration = Duration::from_secs(5);

impl TickFollower {
    /// Register the callback. Frames completed before this call are not seen, so it should
    /// run before the ticker starts where that can be arranged.
    pub fn new(world: &World) -> Result<Self, carla::CarlaError> {
        let mut world = world.clone();
        let queue = Arc::new(TickQueue::new(QUEUE_CAPACITY));
        let sink = queue.clone();
        let callback = world.on_tick(move |snapshot| sink.push(snapshot))?;
        Ok(Self {
            world,
            callback: Some(callback),
            queue,
            last_frame: None,
            frames_processed: 0,
            frames_skipped: 0,
            unreported_skips: 0,
            last_skip_warning: None,
        })
    }

    /// Frames delivered to the caller since this follower started.
    pub fn frames_processed(&self) -> u64 {
        self.frames_processed
    }

    /// Frames that arrived but were passed over for a newer one.
    pub fn frames_skipped(&self) -> u64 {
        self.frames_skipped
    }

    /// The newest frame not yet handled, waiting up to `timeout` for one.
    ///
    /// Same contract as `wait_for_tick_or_timeout`: `Ok(None)` is no frame (the simulation
    /// is paused), `Err` is the server's own error for the caller to classify.
    pub fn next_frame(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<WorldSnapshot>, carla::CarlaError> {
        let drained = self.queue.drain_newest(timeout, self.last_frame);
        if let Some(snapshot) = drained.newest {
            return Ok(Some(self.deliver(snapshot, drained.skipped)));
        }
        debug_assert_eq!(drained.skipped, 0);

        // Nothing queued. Paused or dead? Ask the server; an error is the answer.
        let probed = self.world.wait_for_tick_or_timeout(PROBE_TIMEOUT)?;

        // A frame may have landed during the probe, through either path. Take the newest.
        let drained = self.queue.drain_newest(Duration::ZERO, self.last_frame);
        let mut skipped = drained.skipped;
        let newest = match (drained.newest, probed) {
            (Some(q), Some(p)) if !is_stale(p.frame(), self.last_frame) => {
                if p.frame() > q.frame() {
                    skipped += 1;
                    Some(p)
                } else {
                    if p.frame() < q.frame() {
                        skipped += 1;
                    }
                    Some(q)
                }
            }
            (Some(q), _) => Some(q),
            (None, Some(p)) if !is_stale(p.frame(), self.last_frame) => Some(p),
            (None, _) => None,
        };
        Ok(newest.map(|s| self.deliver(s, skipped)))
    }

    fn deliver(&mut self, snapshot: WorldSnapshot, skipped: u64) -> WorldSnapshot {
        if skipped > 0 {
            self.frames_skipped += skipped;
            self.unreported_skips += skipped;
            let due = self
                .last_skip_warning
                .is_none_or(|at| at.elapsed() >= SKIP_WARNING_INTERVAL);
            if due {
                tracing::warn!(
                    "Skipped {} CARLA frame(s) to stay on the newest (now frame {}); the loop \
                     fell behind the server. {} skipped of {} this session",
                    self.unreported_skips,
                    snapshot.frame(),
                    self.frames_skipped,
                    self.frames_processed + self.frames_skipped + 1,
                );
                self.unreported_skips = 0;
                self.last_skip_warning = Some(Instant::now());
            }
        }
        self.frames_processed += 1;
        self.last_frame = Some(snapshot.frame());
        snapshot
    }
}

impl Drop for TickFollower {
    fn drop(&mut self) {
        if let Some(id) = self.callback.take() {
            // Client-side bookkeeping; fails only if the episode is already gone, and then
            // the callback died with it.
            if let Err(e) = self.world.remove_on_tick(id) {
                tracing::debug!("Could not unregister the on_tick callback: {e}");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, PartialEq)]
    struct MockFrame(usize);

    impl Framed for MockFrame {
        fn frame_number(&self) -> usize {
            self.0
        }
    }

    fn queue_with(frames: &[usize], capacity: usize) -> TickQueue<MockFrame> {
        let q = TickQueue::new(capacity);
        for &f in frames {
            q.push(MockFrame(f));
        }
        q
    }

    #[test]
    fn an_empty_queue_times_out_with_nothing() {
        let q = queue_with(&[], 8);
        let d = q.drain_newest(Duration::from_millis(5), None);
        assert!(d.newest.is_none());
        assert_eq!(d.skipped, 0);
    }

    #[test]
    fn a_single_frame_is_delivered_without_a_skip() {
        let q = queue_with(&[10], 8);
        let d = q.drain_newest(Duration::ZERO, Some(9));
        assert_eq!(d.newest, Some(MockFrame(10)));
        assert_eq!(d.skipped, 0);
    }

    /// The whole point: a backlog collapses to its newest frame and the rest are counted.
    #[test]
    fn a_backlog_drains_to_the_newest_and_counts_the_rest() {
        let q = queue_with(&[11, 12, 13], 8);
        let d = q.drain_newest(Duration::ZERO, Some(10));
        assert_eq!(d.newest, Some(MockFrame(13)));
        assert_eq!(d.skipped, 2);
        // And the queue is empty afterwards.
        let again = q.drain_newest(Duration::ZERO, Some(13));
        assert!(again.newest.is_none());
        assert_eq!(again.skipped, 0);
    }

    /// Frames the queue had to drop are skips too, not silently lost.
    #[test]
    fn frames_pushed_out_of_a_full_queue_count_as_skipped() {
        let q = queue_with(&[1, 2, 3, 4, 5], 3);
        let d = q.drain_newest(Duration::ZERO, None);
        assert_eq!(d.newest, Some(MockFrame(5)));
        // 1 and 2 dropped on push, 3 and 4 passed over.
        assert_eq!(d.skipped, 4);
    }

    /// The liveness probe can deliver a frame before its callback lands; the late copy
    /// must neither be delivered twice nor counted as a skip.
    #[test]
    fn an_already_delivered_frame_is_ignored() {
        let q = queue_with(&[7, 8], 8);
        let d = q.drain_newest(Duration::ZERO, Some(8));
        assert!(d.newest.is_none());
        assert_eq!(d.skipped, 0);
    }

    /// A world reload restarts the frame counter; the new episode's frames are not stale.
    #[test]
    fn a_restarted_frame_counter_is_a_new_episode_not_stale() {
        let q = queue_with(&[3], 8);
        let d = q.drain_newest(Duration::ZERO, Some(4_000_000));
        assert_eq!(d.newest, Some(MockFrame(3)));
    }

    /// A frame pushed from another thread wakes a waiting drain before its timeout.
    #[test]
    fn a_push_wakes_a_waiting_drain() {
        let q = Arc::new(TickQueue::<MockFrame>::new(8));
        let pusher = q.clone();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            pusher.push(MockFrame(42));
        });
        let start = std::time::Instant::now();
        let d = q.drain_newest(Duration::from_secs(5), None);
        t.join().unwrap();
        assert_eq!(d.newest, Some(MockFrame(42)));
        assert!(start.elapsed() < Duration::from_secs(2));
    }
}
