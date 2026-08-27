# 017 — The CARLA server segfaults under long uptime

**Severity**: Medium
**Component**: CARLA 0.9.16 server (`CarlaUE4-Linux-Shipping`), not `acb_bridge`
**Status**: Open — observed and characterised, cause not investigated

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
