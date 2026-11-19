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

from carla_map_gen import georef as G
from carla_map_gen.postprocess import postprocess_osm
from carla_map_gen.repair import repair_map

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
    parser.add_argument("--no-repairs", action="store_true",
                        help="skip the lanelet2 repairs (carla_map_gen.repair), which run by default")
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
        # crdesigner converts the file with the towns' georeference (its own geo handling of a real CRS
        # is not trusted); relocate() then puts every node where the file's CRS says (roadmap 087)
        georef = G.GeoRef(odr_path)
        with tempfile.TemporaryDirectory() as tmp:
            town_odr = os.path.join(tmp, f"{map_name}.xodr")
            G.rewrite_town_proj(odr_path, town_odr)
            opendrive_to_lanelet(town_odr, osm_path, odr_config=odr_config,
                                 lanelet2_config=lanelet2_config)

            # Post-process
            postprocess_osm(osm_path)
            info, shift = G.relocate(osm_path, georef, output_dir)
            print(f"Projector: {info.splitlines()[0].split(': ')[1]}"
                  + (f", map frame = OpenDRIVE frame + ({shift[0]:.3f}, {shift[1]:.3f}) (map_frame.yaml)"
                     if shift != (0.0, 0.0) else ", map frame = OpenDRIVE frame"))
            (output_dir / "map_config.yaml").write_text(MAP_CONFIG_YAML)

            if not args.no_repairs:
                rep_odr = odr_path
                if shift != (0.0, 0.0):
                    rep_odr = os.path.join(tmp, f"{map_name}_map_frame.xodr")
                    G.shift_xodr(odr_path, rep_odr, shift)
                print("Repairing (carla_map_gen.repair)...")
                repair_map(output_dir, xodr=rep_odr)
    finally:
        if cleanup_odr:
            os.unlink(odr_path)

    # Summary
    print(f"\nGenerated files in {output_dir}/:")
    for f in sorted(output_dir.iterdir()):
        size = f.stat().st_size
        print(f"  {f.name} ({size:,} bytes)")



if __name__ == "__main__":
    main()
