# 020 — Attaching one more node to a running stack fails obscurely

**Severity**: Medium
**Component**: `config/cyclonedds-localhost.xml` (parent repository)
**Status**: Fixed by raising `MaxAutoParticipantIndex` to 450

## Symptom

Any new ROS process joining the ego stack's domain -- a probe, `ros2 topic echo`, a
diagnostic node -- dies at node creation:

```
RCLError: error creating node: rcl node's rmw handle is invalid, at ./src/rcl/node.c:415
[rcl]: Failed to fini publisher for node: 1
terminate called without an active exception
```

Nothing in that names the cause, and the last line is a crash rather than an error. It arrives
only when the stack is large, so it looks like an intermittent fault in whatever was being
attached. It cost an hour of debugging a measurement script that was not broken.

## Cause

`CYCLONEDDS_TRACE=1` says it in one line:

```
Failed to find a free participant index for domain 1
```

`MaxAutoParticipantIndex` was 300, and the domain had 300 participants. **Every process takes
an index**, not every node: the ego stack alone runs about 158 `component_node` processes,
plus its bridge, the scenario stack, the interpreter, and any tool already attached. Measured
at the point of failure, 600 ports were in use across 8410-9009 -- exactly 300 indices at
`ParticipantGain` 2.

The comment on the setting had assumed "the full stack runs ~170", which was true when it was
written.

## Fix

Raised to 450. The ceiling is the next domain's port base: with `DomainGain` 1000, domain *d*
starts at 7400 + 1000*d* and its participant ports run from base + 10 to base + 11 + 2*N*, so
*N* must stay below 494 for domain 1 to keep clear of domain 2's 9400. Domain 2 matters here --
it is where background AVs run.

Verified by attaching a node to a freshly started stack: it connects, and 318 of the 900
available ports are in use.

## The part that is easy to get wrong

**Every participant must agree on this number.** Cyclone's unicast discovery probes peer ports
for indices 0 to `MaxAutoParticipantIndex`. A process that joins with a *higher* cap than the
running stack can be granted an index the stack never probes, so it comes up cleanly, sees
nothing, and is seen by nothing. Changing this value therefore means restarting everything that
was already running -- raising it and attaching to a stack that is still up produces a silent
failure in place of a loud one.

## Worth noting

The underlying pressure is that ~158 separate `component_node` processes each take a
participant. Running those as composable nodes inside containers would cut the count by an
order of magnitude. That is a launch-architecture change, and is not attempted here.
