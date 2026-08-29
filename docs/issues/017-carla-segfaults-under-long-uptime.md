# 017 — The CARLA server segfaults under long uptime

**Severity**: Medium
**Component**: CARLA 0.9.16 server (`CarlaUE4-Linux-Shipping`), not `acb_bridge`
**Status**: Open — the crash is now captured and the bad restart is fixed; the segfault's own cause is still uninvestigated

## What happens

The server exits on SIGSEGV with a core dump, taking every client with it:

```
Engine crash handling finished; re-raising signal 11 for the default handler. Good bye.
Segmentation fault (core dumped)
error: recipe `run-carla` failed with exit code 139
```

Seen twice on 2026-08-25/26, at roughly **2 days** and **15 hours** of server uptime. Both
times the machine was otherwise healthy — 90 GB of memory free, no actor leak (0 sensors,
1 stray vehicle), and `carla.log` at its 56-byte startup banner, so this is not the
orphaned-stream storm of [015](015-sensors-destroyed-while-still-listening.md).

## Why it matters here

It is not a simulation-fidelity problem; it is an **unattended-running** problem. Both
crashes landed mid-experiment and each cost an entire arm of a comparison:

- The first killed a stack partway through an interleaved A/B, leaving one arm short of
  data. A missing arm is worse than a halted run: it looks like a result.
- The second killed the adapter's connection, which then reported `CARLA not ready:
  time-out of 30000ms` and skipped the stack silently.

Anything that runs a batch of scenarios without a person watching will hit this.

## What is known

- **Not memory pressure.** 87–97 GB free at both crashes.
- **Not accumulated actors.** A census during the second period: 174 actors, of which 115
  static map props and 57 traffic lights; 0 sensors and 1 vehicle.
- **Not the stream storm.** `carla.log` stayed at 56 bytes.
- **Uptime is the only common factor**, and the two samples differ by more than an order of
  magnitude in hours, so "uptime" is a correlation with n = 2 rather than a threshold.

Restarting the server clears it completely; a fresh server has never crashed early.

## What to do about it

The cheap mitigation is in the harness rather than the server: **check CARLA is alive before
each stack and stop rather than continue**. A crash that halts a run is recoverable; a crash
that silently removes data from one arm of a comparison is not. `scripts/run_scenarios.sh`
does this.

Restarting CARLA between long batches is the other obvious mitigation, and costs about a
minute.

## What has not been done

No attempt to find the cause. The core dumps were not kept, the crash was not reproduced
deliberately, and no CARLA-side logs beyond the banner were examined. If this becomes worth
chasing, start by preserving a core and reading `CarlaUE4/Saved/Logs/`.

## The "third failure mode" was a measurement error (2026-08-27)

An earlier version of this section claimed a third way for the server to fail: alive,
answering RPC, and holding an empty world. The evidence was a census that read

```
total actors: 0     Counter()
sync: True   dt: 0.05
```

on a server that had been up about 2.5 days, where a fresh Town01 carries 173. It was written
up as the worst mode yet, because nothing announces it.

**It was not a server fault. It was the census.** In synchronous mode a client's actor list
comes from the episode snapshot, and a client that has never observed a tick has no snapshot.
It reports zero actors and frame zero, on a server that is perfectly healthy. Measured
directly -- one server, one instant, both clients in sync mode:

```
sync, no ticker:
   old client (has seen ticks): 174 actors, snapshot frame 4345539
   new client (never ticked)  : 0 actors, snapshot frame 0
```

The census script was a fresh client every time, and the bridge that would normally have been
ticking had been killed, so the server sat in the sync mode the bridge left behind. The
`sync: True` in the original output said so; it was printed and not read.

Two CARLA restarts were performed on the strength of that reading, and neither was needed.

### What this costs, and what to do instead

`get_settings()`, `get_map()` and `get_server_version()` are direct RPCs and answer correctly
for any client, tick or no tick. Only the snapshot-derived views -- the actor list, the frame
number -- need a tick the client has seen.

So a health check must read the mode first:

- **Asynchronous**: the actor count is meaningful. Zero actors is a genuine fault, since a
  loaded map has static props and a spectator the server creates itself.
- **Synchronous**: the actor count from a fresh client means nothing. Liveness is whether the
  direct RPCs answer. Do not tick to find out -- the ticker belongs to whoever put the server
  in this mode, and a second ticker corrupts their run.

`scripts/carla_health.py` implements exactly that, and `just scenario` refuses to start
without it.

### The genuine incidents in this issue are unaffected

The two crashes documented above were real: the process died and left a core. A third real
death followed on 2026-08-27, where the bridge itself -- not a passive probe -- reported
`CARLA connection looks dead at Initialize` and the systemd unit showed the server had
restarted underneath a running batch. Those remain the subject of this issue.

Note also that a wedged server may ignore `SIGTERM` and need `SIGKILL`, and that
`just carla-stop` reports "CARLA is not running on port 2000" for an instance started by hand
rather than as the systemd user unit.

## Caught in the act: the crash, and the restart that made it worse (2026-08-29)

This issue has said since it was written that no crash was ever captured -- no core kept, no
server log read. One was finally caught, and the journal had it all along:

```
13:51:42  Signal 11 caught.
13:51:43  CommonUnixCrashHandler: Signal=11
13:51:43  Engine crash handling finished; re-raising signal 11 for the default handler.
13:51:43  Segmentation fault (core dumped)
13:51:43  carla-run-2000.service: Main process exited, code=exited, status=139/n/a
13:51:43  carla-run-2000.service: Consumed 4d 15h 16min 6.717s CPU time.
13:51:53  carla-run-2000.service: Scheduled restart job, restart counter is at 1.
13:51:56  No protocol specified
```

So the segfault is real, UE4's own crash handler runs, and `journalctl --user -u
carla-run-2000` is where to look. That is a better source than the core, which the kernel did
not keep -- `coredumpctl` is empty and `/var/lib/systemd/coredump` has nothing, so whatever
"(core dumped)" wrote went somewhere the default limits discarded.

Note the CPU time: **4 days 15 hours** consumed by that instance. The uptime correlation this
issue is named for now has a third data point.

### The restart is worse than the crash

`Restart=on-failure` brought CARLA back ten seconds later, and the replacement never served a
single RPC. Twenty-six minutes after it started:

```
process   alive, 158 threads, 2.4 GB RSS, 121% CPU, one thread R and the rest sleeping
port      LISTEN 36  -- thirty-six connections completed by the kernel, never accepted
clients   both carla_scenario_bridge and acb_bridge timing out at 30 s, forever
```

The bridges' logs are worth reading together, because separately each looks like its own bug:

```
acb  WARN CARLA not ready: Simulation error: time-out of 30000ms while waiting for the simulator
csb  WARN Returning error: Failed to enable sync mode: get settings
```

An hour was spent treating the acb line as a reconnect defect -- a fresh CARLA client's first
`GetWorld()` waits for a world snapshot, so it genuinely cannot connect to a server nobody is
ticking, and that was a plausible story. It was wrong. csb was failing at the same instants on
a call that needs no tick at all, which is what rules the theory out: the server was serving
nobody.

A deliberate `just carla-stop && just carla-start` produced a working server immediately, so
the environment was fine. Only the automatic restart produced the zombie.

That is the part worth fixing. A crash leaves a state anything can detect. This restart
converts it into "process up, port listening, RPC dead", which passes every check except a real
client call -- and it is the state a long unattended batch will meet.

### Fix: the unit is not "up" until it serves

`scripts/carla_wait_ready.sh` runs as the unit's `ExecStartPost` and polls `carla_health.py`
until CARLA answers, failing after `CARLA_READY_TIMEOUT` (180 s default, with
`TimeoutStartSec=300` above it). A unit whose `ExecStartPost` fails is stopped by systemd, so a
restart that produces a server nobody can talk to ends as "down" -- which `just carla-health`
reports plainly and which `just scenario`, `just ego-av` and `just bg-av` already refuse to run
against.

Verified both ways: a healthy start reaches `active` only after the server answers
(`Result=success`), and the gate against a port with nothing behind it exits non-zero with the
reason.

`Restart=on-failure` is kept. A restart that works is still worth having; this only stops a
restart that does not from pretending otherwise.

### Keeping a core: attempted, and it cannot be done this way

The obvious next step was to keep the next core and read it. It does not work, and the reason
is worth writing down so nobody spends the afternoon again.

Three things were checked, and two of them were fixed:

1. **The unit's soft core limit was 0** (hard was already infinity), so nothing would have been
   written whatever else was true. `carla_start.sh` now passes `LimitCORE=infinity`, and the
   unit reports `LimitCORESoft=infinity`.
2. **`core_pattern` pipes to apport**, not systemd-coredump, which is not installed here.
   apport is enabled and demonstrably working -- `/var/crash` holds recent reports for `ros2`,
   `rviz2` and `python3`. So the pipeline is live and needs no root.
3. **Unreal disables core dumps for itself.** This is the blocker. The running process reports

   ```
   Max core file size        0                    unlimited
   ```

   while its parent shell has `unlimited`, so the engine lowers it after exec. The binary
   contains the strings `Disabling core dumps.` and `Alternatively, pass -nocore if you are
   unable or unwilling to do that.`, which implies a switch. There is one, and it does not
   help: started with `-core` present in `/proc/<pid>/cmdline`, the process still comes up with
   a soft limit of 0. This Shipping build disables cores unconditionally.

So a kernel core is unavailable without patching the engine. `CARLA_EXTRA_ARGS` was added to
`run.sh` while testing this and is kept -- it is the place to pass an engine switch without
editing the launcher.

**What is left if the cause is ever worth chasing**: run the server under a debugger that
outlives it (`gdb --batch -ex run -ex bt --args ...`), or attach a ptrace-based dumper to the
live process. Both change how the server runs, so neither belongs in the default path. The
journal already gives the signal, the timing and the CPU time consumed, which is what the three
data points in this issue rest on.

### Still not done

The cause of the segfault itself.
