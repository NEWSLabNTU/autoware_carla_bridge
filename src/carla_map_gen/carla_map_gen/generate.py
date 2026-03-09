"""Generate Autoware-compatible Lanelet2 maps from CARLA.

Supports two modes:
  - Live: Connect to a running CARLA server and extract the OpenDRIVE map
  - Offline: Use a local .xodr file directly (no CARLA server needed)
"""

import argparse
import os
import tempfile
from pathlib import Path

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
        description="Generate Autoware Lanelet2 map from CARLA")
    parser.add_argument("--host", default="localhost",
                        help="CARLA server host (default: localhost)")
    parser.add_argument("--port", type=int, default=2000,
                        help="CARLA server port (default: 2000)")
    parser.add_argument("--xodr",
                        help="Path to .xodr file (offline mode, no CARLA needed)")
    parser.add_argument("--output-dir", required=True,
                        help="Output directory for generated map files")
    args = parser.parse_args()

    if args.xodr:
        # Offline mode: use local .xodr file
        xodr_path = Path(args.xodr)
        if not xodr_path.exists():
            raise FileNotFoundError(f"OpenDRIVE file not found: {xodr_path}")
        map_name = xodr_path.stem  # e.g. "Town01" from "Town01.xodr"
        odr_path = str(xodr_path)
        cleanup_odr = False
        print(f"Using OpenDRIVE file: {xodr_path}")
    else:
        # Live mode: connect to CARLA
        import carla
        print(f"Connecting to CARLA at {args.host}:{args.port}...")
        client = carla.Client(args.host, args.port)
        client.set_timeout(30.0)

        world = client.get_world()
        carla_map = world.get_map()

        # Extract map name (e.g. "Town01" from "Carla/Maps/Town01")
        map_name = carla_map.name.split("/")[-1]

        # Write OpenDRIVE to temp file
        odr_string = carla_map.to_opendrive()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xodr",
                                         delete=False) as f:
            f.write(odr_string)
            odr_path = f.name
        cleanup_odr = True

    print(f"Map: {map_name}")

    output_dir = Path(args.output_dir)

    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output: {output_dir}")

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
        if cleanup_odr:
            os.unlink(odr_path)

    # Write config files
    (output_dir / "map_config.yaml").write_text(MAP_CONFIG_YAML)
    (output_dir / "map_projector_info.yaml").write_text(MAP_PROJECTOR_INFO_YAML)

    # Summary
    print(f"\nGenerated files in {output_dir}/:")
    for f in sorted(output_dir.iterdir()):
        size = f.stat().st_size
        print(f"  {f.name} ({size:,} bytes)")



if __name__ == "__main__":
    main()
