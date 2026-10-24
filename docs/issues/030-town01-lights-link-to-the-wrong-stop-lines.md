# 030: Town01's lanelets link most traffic lights to the wrong stop line

**Severity**: High (stand-alone, the ego drives through CARLA's red). **Status**: Fixed in the
map data by scenario-world-bridge's `scripts/repair_lanelet_signal_links.py`; nothing in acb
changed. Found 2026-10-24 (scenario-world-bridge roadmap 051, "the manual scene" acceptance).

## What happens

The manual scene: CARLA synchronous, ticked in real time by a plain client; a Model 3 hero at
spawn point 74; `generate_traffic.py`; `just autoware map:=Town01`; `auto_drive` to
scenario-world-bridge's `scenarios/ego_poses.yaml`. The ego arrives, but at the junction near
x = 168 (CARLA light 644, OpenDRIVE signal 395) it crossed CARLA's stop line on red at
4.17 m/s without slowing. At the next junction (light 630) it stopped, about 8.6 m past
CARLA's stop waypoint.

acb is not the cause. The signals reach Autoware: acb publishes
`/perception/traffic_light_recognition/external/traffic_signals` from
`carla/traffic_lights.yaml`, 36 lights in 36 regulatory elements, every light resolved. Each
element's light way is the right CARLA light. What is wrong is the map's lanelet links, in
the TUM pack as shipped:

- the ego's approach lanelet before light 644 (18715) references no traffic light. That
  light's element (43860) is referenced by lanelet 19033, whose stop line lies across the
  next lane, 4 m to the side;
- across the town, the stop line of 12 of the 36 elements is 25 to 214 m from the stop
  waypoint CARLA gives for its light, at another junction. Element 43910 (light 633, pole at
  x = 103) has its stop line at x = 326. Five more lie a lane to the side;
- only light 630's approach was linked right, and its stop line was 8.6 m past CARLA's.

The same defect was found and fixed on Town05 for roadmap 031 (defect 8). Town01 had never
been run through that repair.

## The fix

Run `scripts/repair_lanelet_signal_links.py Town01 --port <CARLA port>` from
scenario-world-bridge (`fe33862` or later), with CARLA holding Town01 and something ticking
it. The script linked 38 lanelets and gave each element a stop line across its approach.
It keeps the file it started from as `lanelet2_map.osm.orig-signal-links`. Two checks pass
afterwards:

- each CARLA light's approach lies in a road lanelet that runs the light's way and
  references the light's element: 36 of 36 (28 failed in the shipped map);
- each such lanelet's centre line crosses its stop line: 36 of 36.

The first version of the repair had two bugs of its own, both found here:

- Its heading test was unsigned. Town01's two-lane roads share one middle line, and where
  CARLA's stop waypoint lies on that line, the test picked the oncoming lanelet about half
  the time. Light 373 went to the westbound lane, and in scenario-world-bridge's
  `town01_i3_red_light` the ego drove through SSv2's red at 4.2 m/s.
- On roads where the map's lanes sit 2.2 m off CARLA's, the stop line only spanned CARLA's
  lane, and it missed Autoware's path by 0.2 m.

Restart the vehicle stack after the repair: a stack started earlier keeps the old map, and
did so here once.

The map pack lives under `data/`, which git ignores, so every host has to run the repair
itself. acb's README says so.

## After the repair

The manual scene stops for CARLA's red at its stop line and leaves on green.
`town01_traffic_light` and `town01_i3_red_light` stop for SSv2's red at CARLA's line, 2.6 to
2.8 m from the centre of the car.

Not fixed by this: on the default clustering stack the ego is still held at Town01's
south-east corner by the street lamp of issue 029, which `town01_i3_red_light` drives past
after its light. The suite runs the incidents on the CenterPoint stack, which does not
report the lamp.
