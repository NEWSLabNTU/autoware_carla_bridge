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
