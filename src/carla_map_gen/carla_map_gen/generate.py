"""Generate Autoware-compatible Lanelet2 maps from a running CARLA server."""

import argparse
import os
import tempfile
from pathlib import Path

import carla
from crdesigner.common.config.lanelet2_config import Lanelet2Config
from crdesigner.common.config.opendrive_config import OpenDriveConfig
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet

from carla_map_gen.postprocess import postprocess_osm

MAP_CONFIG_YAML = """\
/**:
  ros__parameters:
    map_origin:
      latitude: 0.0
      longitude: 0.0
      elevation: 0.0
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
"""

MAP_PROJECTOR_INFO_YAML = """\
projector_type: local
vertical_datum: WGS84
mgrs_grid: 31NAA
"""


def main():
    parser = argparse.ArgumentParser(
        description="Generate Autoware Lanelet2 map from a running CARLA server")
    parser.add_argument("--host", default="localhost",
                        help="CARLA server host (default: localhost)")
    parser.add_argument("--port", type=int, default=2000,
                        help="CARLA server port (default: 2000)")
    parser.add_argument("--output-dir",
                        help="Output directory (default: auto-detect from map name)")
    parser.add_argument("--project-dir",
                        help="Project root directory (default: auto-detect)")
    args = parser.parse_args()

    # Connect to CARLA
    print(f"Connecting to CARLA at {args.host}:{args.port}...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    world = client.get_world()
    carla_map = world.get_map()

    # Extract map name (e.g. "Town01" from "/Game/Carla/Maps/Town01/Town01")
    map_name = carla_map.name.split("/")[-1]
    print(f"Map: {map_name}")

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        project_dir = Path(args.project_dir) if args.project_dir else _find_project_dir()
        output_dir = project_dir / "data" / "carla-autoware-bridge" / map_name

    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output: {output_dir}")

    # Extract OpenDRIVE and write to temp file
    odr_string = carla_map.to_opendrive()
    with tempfile.NamedTemporaryFile(mode="w", suffix=".xodr", delete=False) as f:
        f.write(odr_string)
        odr_path = f.name

    try:
        # Convert OpenDRIVE to Lanelet2
        osm_path = str(output_dir / "lanelet2_map.osm")

        lanelet2_config = Lanelet2Config()
        lanelet2_config.autoware = True
        lanelet2_config.use_local_coordinates = True

        odr_config = OpenDriveConfig()

        print("Converting OpenDRIVE to Lanelet2...")
        opendrive_to_lanelet(odr_path, osm_path, odr_config=odr_config,
                             lanelet2_config=lanelet2_config)

        # Post-process
        postprocess_osm(osm_path)
    finally:
        os.unlink(odr_path)

    # Write config files
    (output_dir / "map_config.yaml").write_text(MAP_CONFIG_YAML)
    (output_dir / "map_projector_info.yaml").write_text(MAP_PROJECTOR_INFO_YAML)

    # Summary
    print(f"\nGenerated files in {output_dir}/:")
    for f in sorted(output_dir.iterdir()):
        size = f.stat().st_size
        print(f"  {f.name} ({size:,} bytes)")


def _find_project_dir() -> Path:
    """Find project root by walking up from this file's location."""
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "justfile").exists():
            return parent
    raise RuntimeError("Could not find project root (no justfile found)")


if __name__ == "__main__":
    main()
