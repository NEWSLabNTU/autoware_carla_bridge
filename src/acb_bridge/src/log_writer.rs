//! Write log lines from a thread of their own, so a slow disk cannot stall the bridge.
//!
//! The bridge's stdout and stderr are files -- play_launch opens `play_log/.../out` and
//! hands them over -- and on this project's host those files live on a busy spinning disk
//! (ext4, `/home`). A `write()` to such a file normally returns at once into the page cache,
//! but it updates the inode's mtime, which needs a journal handle, and while jbd2 is
//! committing on a saturated disk that handle waits: the thread sits in uninterruptible
//! sleep in `wait_transaction_locked` until the commit finishes.
//!
//! Measured live (2026-09-29, the main thread sampled every 20 ms), that wait was 0.5 to
//! 3 s several times a scenario on an ordinary day, and up to 25 s while another user's
//! `rm -rf` saturated the disk. It happened on the main loop, which logged a line for
//! every `/tf_static` message -- about 40 a second, because SSv2 republishes its `ego` and
//! `entities` frames there at the frame rate. For as long as the write was stuck, the loop
//! published no velocity, steering or ground truth and drained no frames; it came back to
//! "Skipped 19-62 CARLA frames", and gyro_odometer, which had seen no vehicle twist for
//! that long, had already raised its timeout and the MRM handler an emergency stop.
//! play_launch's own sampler, writing to the same disk, shows a gap at exactly the same
//! instants, which is how this was first told apart from anything in the bridge.
//!
//! So nothing on a hot path writes to the file any more. [`NonBlockingLog`] hands each
//! formatted line to a bounded queue and returns; one background thread writes them out.
//! When the disk stalls, that thread stalls, the queue fills, and further lines are dropped
//! and counted -- a log is worth less than a control loop -- and the count is written as
//! soon as the disk comes back, so a gap in the log says so rather than hiding.

use std::{
    io::{self, Write},
    sync::{
        atomic::{AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc,
    },
    thread::JoinHandle,
};

/// Lines held while the disk is stalled. At the bridge's usual few lines a second this
/// covers minutes; a burst of warnings during a long stall is what it is sized for.
pub const QUEUE_LINES: usize = 4096;

/// A `MakeWriter` for tracing-subscriber whose writes never block on I/O.
#[derive(Clone)]
pub struct NonBlockingLog {
    tx: SyncSender<Vec<u8>>,
    dropped: Arc<AtomicU64>,
    /// Lines queued and not yet written, so shutdown can wait for exactly those.
    pending: Arc<AtomicU64>,
}

/// Joins the writer thread on drop, after everything queued has been written.
pub struct LogGuard {
    thread: Option<JoinHandle<()>>,
    pending: Arc<AtomicU64>,
}

impl NonBlockingLog {
    /// A log writing to `sink` from a thread named `log-writer`.
    pub fn new<W: Write + Send + 'static>(sink: W, capacity: usize) -> (Self, LogGuard) {
        let (tx, rx) = mpsc::sync_channel(capacity.max(1));
        let dropped = Arc::new(AtomicU64::new(0));
        let pending = Arc::new(AtomicU64::new(0));
        let thread = std::thread::Builder::new()
            .name("log-writer".into())
            .spawn({
                let dropped = dropped.clone();
                let pending = pending.clone();
                move || drain(rx, sink, &dropped, &pending)
            })
            .expect("cannot spawn the log writer thread");
        (
            Self {
                tx,
                dropped,
                pending: pending.clone(),
            },
            LogGuard {
                thread: Some(thread),
                pending,
            },
        )
    }

    /// Lines dropped because the queue was full, not yet reported in the log.
    #[cfg(test)]
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn send(&self, line: Vec<u8>) {
        self.pending.fetch_add(1, Ordering::SeqCst);
        match self.tx.try_send(line) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                self.pending.fetch_sub(1, Ordering::SeqCst);
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
            // The writer is gone (shutdown): there is nowhere left to write.
            Err(TrySendError::Disconnected(_)) => {
                self.pending.fetch_sub(1, Ordering::SeqCst);
            }
        }
    }
}

/// The writer thread: write each line, and before it any count of lines dropped since the
/// last one written.
fn drain<W: Write>(rx: Receiver<Vec<u8>>, mut sink: W, dropped: &AtomicU64, pending: &AtomicU64) {
    for line in rx {
        let lost = dropped.swap(0, Ordering::Relaxed);
        if lost > 0 {
            let _ = writeln!(
                sink,
                "[acb_bridge log] {lost} line(s) dropped: the log file blocked and the queue \
                 filled (the bridge itself kept running)"
            );
        }
        let _ = sink.write_all(&line);
        let _ = sink.flush();
        pending.fetch_sub(1, Ordering::SeqCst);
    }
    let lost = dropped.swap(0, Ordering::Relaxed);
    if lost > 0 {
        let _ = writeln!(sink, "[acb_bridge log] {lost} line(s) dropped at shutdown");
        let _ = sink.flush();
    }
}

impl Drop for LogGuard {
    fn drop(&mut self) {
        // The global subscriber keeps a sender for the life of the process, so the thread
        // never sees the channel close. Wait for what is queued to be written instead, and
        // bound even that: a disk stalled at exit must not hang the shutdown.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while self.pending.load(Ordering::SeqCst) > 0 && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        if let Some(thread) = self.thread.take() {
            if thread.is_finished() {
                let _ = thread.join();
            }
        }
    }
}

/// One event's worth of output. tracing-subscriber formats an event into a buffer and
/// writes it whole, so each `LineWriter` normally carries exactly one line.
pub struct LineWriter {
    log: NonBlockingLog,
    buf: Vec<u8>,
}

impl Write for LineWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.buf.extend_from_slice(data);
        Ok(data.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for LineWriter {
    fn drop(&mut self) {
        if !self.buf.is_empty() {
            self.log.send(std::mem::take(&mut self.buf));
        }
    }
}

impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for NonBlockingLog {
    type Writer = LineWriter;

    fn make_writer(&'a self) -> Self::Writer {
        LineWriter {
            log: self.clone(),
            buf: Vec::with_capacity(256),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        sync::{Condvar, Mutex},
        time::{Duration, Instant},
    };

    /// A sink that records what it is given and can be made to block, like a file on a
    /// disk whose journal is committing.
    /// Whether writes block, and what has been written.
    type Gate = Arc<(Mutex<(bool, Vec<u8>)>, Condvar)>;

    #[derive(Clone, Default)]
    struct GatedSink {
        state: Gate,
        /// Set once a write has started, blocked or not.
        writing: Arc<std::sync::atomic::AtomicBool>,
    }

    impl GatedSink {
        fn set_blocked(&self, blocked: bool) {
            let (m, cv) = &*self.state;
            m.lock().unwrap().0 = blocked;
            cv.notify_all();
        }
        fn writing(&self) -> bool {
            self.writing.load(Ordering::SeqCst)
        }
        fn text(&self) -> String {
            String::from_utf8(self.state.0.lock().unwrap().1.clone()).unwrap()
        }
    }

    impl Write for GatedSink {
        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            self.writing.store(true, Ordering::SeqCst);
            let (m, cv) = &*self.state;
            let mut s = cv.wait_while(m.lock().unwrap(), |s| s.0).unwrap();
            s.1.extend_from_slice(data);
            Ok(data.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn log_line(log: &NonBlockingLog, text: &str) {
        use tracing_subscriber::fmt::MakeWriter;
        let mut w = log.make_writer();
        w.write_all(text.as_bytes()).unwrap();
    }

    fn wait_for(cond: impl Fn() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !cond() {
            assert!(Instant::now() < deadline, "timed out");
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn lines_reach_the_sink_in_order() {
        let sink = GatedSink::default();
        let (log, _guard) = NonBlockingLog::new(sink.clone(), 16);
        for i in 0..5 {
            log_line(&log, &format!("line {i}\n"));
        }
        wait_for(|| sink.text().lines().count() == 5);
        assert_eq!(sink.text(), "line 0\nline 1\nline 2\nline 3\nline 4\n");
    }

    /// The whole point: a sink stuck for as long as the disk stalls does not stall the
    /// caller. Lines beyond the queue are dropped, counted, and reported afterwards.
    #[test]
    fn a_blocked_sink_never_blocks_the_logger() {
        let sink = GatedSink::default();
        sink.set_blocked(true);
        let (log, _guard) = NonBlockingLog::new(sink.clone(), 4);

        // Let the writer take the first line and stick in it, as it would in a stalled
        // `write()`. Until then a drop could be reported (and counted) before the flood.
        log_line(&log, "line 0\n");
        wait_for(|| sink.writing());

        let start = Instant::now();
        for i in 1..100 {
            log_line(&log, &format!("line {i}\n"));
        }
        assert!(
            start.elapsed() < Duration::from_millis(200),
            "logging blocked for {:?}",
            start.elapsed()
        );
        // One line in the writer's hands, four in the queue; the other 95 are dropped.
        assert_eq!(log.dropped(), 95);

        sink.set_blocked(false);
        wait_for(|| log.pending.load(Ordering::SeqCst) == 0);
        log_line(&log, "after\n");
        wait_for(|| sink.text().contains("after"));
        // The notice goes out with the next line the writer takes, which is the queued
        // line 1: the drops happened while it waited, so this is within a queue's length
        // of where the gap is, and it says how many.
        assert_eq!(
            sink.text(),
            "line 0\n[acb_bridge log] 95 line(s) dropped: the log file blocked and the queue \
             filled (the bridge itself kept running)\nline 1\nline 2\nline 3\nline 4\nafter\n"
        );
    }

    /// Dropping the guard at shutdown writes out what is still queued.
    #[test]
    fn the_guard_flushes_queued_lines_on_drop() {
        let sink = GatedSink::default();
        sink.set_blocked(true);
        let (log, guard) = NonBlockingLog::new(sink.clone(), 16);
        for i in 0..3 {
            log_line(&log, &format!("line {i}\n"));
        }
        let releaser = {
            let sink = sink.clone();
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(50));
                sink.set_blocked(false);
            })
        };
        drop(guard);
        releaser.join().unwrap();
        assert_eq!(sink.text(), "line 0\nline 1\nline 2\n");
    }

    /// Each event arrives as one line, even though fmt may write it in pieces.
    #[test]
    fn a_line_written_in_pieces_is_sent_once() {
        use tracing_subscriber::fmt::MakeWriter;
        let sink = GatedSink::default();
        let (log, _guard) = NonBlockingLog::new(sink.clone(), 16);
        {
            let mut w = log.make_writer();
            w.write_all(b"part one, ").unwrap();
            w.write_all(b"part two\n").unwrap();
        }
        wait_for(|| sink.text().ends_with('\n'));
        assert_eq!(sink.text(), "part one, part two\n");
    }
}
