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

`scripts/repair_lanelet_signal_links.py Town01 --port <CARLA port>` in scenario-world-bridge,
with CARLA holding Town01 and ticking. It changed the links of 41 lanelets and gave every
element a stop line at its light's stop waypoints. Afterwards every stop waypoint is within
2.0 m of its element's stop line, which is the half-width of a lane. The script keeps the
file before it as `lanelet2_map.osm.orig-signal-links`. The map pack lives under `data/`,
which git ignores, so every host has to run the repair itself. acb's README says so.

After the repair, the same scene: see scenario-world-bridge's roadmap 051, K5.
