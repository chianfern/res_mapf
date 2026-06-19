# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Generate a LIF-style .json file from grid specifications.
# TODO: update to final LIF format.

Example:
python3 generate_lif_from_grid.py --width 7 --height 7 --spacing 1.0 --map-id basic_grid  -o basic_grid.json

"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def parse_obstacles(raw: str | None) -> set[tuple[int, int]]:
    """
    Parse obstacle coordinates from a string like "3,3 4,3 3,4"
    into a set of (x, y) tuples.
    """
    if not raw:
        return set()

    obstacles: set[tuple[int, int]] = set()

    for token in raw.split():
        try:
            x_str, y_str = token.split(",")
            obstacles.add((int(x_str), int(y_str)))
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid obstacle coordinate '{token}', expected format 'x,y'"
            ) from exc

    return obstacles


def generate(
    width: int,
    height: int,
    *,
    obstacles: set[tuple[int, int]] | None = None,
    spacing: float = 1.0,
    map_id: str = "basic_grid",
    map_version: str = "1.0",
    allowed_deviation_xy: float = 0.5,
    allowed_deviation_theta: float = 0.1,
    max_speed: float = 1.0,
) -> dict:
    """
    Generate a 4-connected grid map.

    node: P_3_4
    edge: E_3_4_to_4_4
    """

    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive integers")

    if obstacles is None:
        obstacles = set()

    out_of_bounds = {
        o for o in obstacles if not (0 <= o[0] < width and 0 <= o[1] < height)
    }
    if out_of_bounds:
        raise ValueError(f"Obstacles out of grid bounds: {sorted(out_of_bounds)}")

    nodes = []
    edges = []
    node_lookup: dict[tuple[int, int], str] = {}

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------

    for y in range(height):
        for x in range(width):
            if (x, y) in obstacles:
                continue

            node_id = f"P_{x}_{y}"
            node_lookup[(x, y)] = node_id

            nodes.append(
                {
                    "node_id": node_id,
                    "x": float(x * spacing),
                    "y": float(y * spacing),
                    "theta": 0.0,
                    "allowed_deviation_xy": allowed_deviation_xy,
                    "allowed_deviation_theta": allowed_deviation_theta,
                    "map_description": "",
                }
            )

    # ------------------------------------------------------------------
    # Edges
    # ------------------------------------------------------------------

    directions = [
        (1, 0),  # right
        (0, 1),  # down
    ]

    for y in range(height):
        for x in range(width):
            if (x, y) not in node_lookup:
                continue

            start_node_id = node_lookup[(x, y)]

            for dx, dy in directions:
                nx = x + dx
                ny = y + dy

                if (nx, ny) not in node_lookup:
                    continue

                end_node_id = node_lookup[(nx, ny)]

                edges.append(
                    {
                        "edge_id": f"E_{x}_{y}_to_{nx}_{ny}",
                        "start_node_id": start_node_id,
                        "end_node_id": end_node_id,
                        "bidirectional": True,
                        "max_speed": max_speed,
                        "length": spacing,
                    }
                )

    return {
        "map_info": {
            "map_id": map_id,
            "map_version": map_version,
            "map_status": "ENABLED",
            "map_descriptor": f"{width}x{height} grid",
        },
        "nodes": nodes,
        "edges": edges,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a LIF-style .json of a grid map.",
    )

    parser.add_argument(
        "--width", type=int, required=True, help="Grid width (number of columns)"
    )
    parser.add_argument(
        "--height", type=int, required=True, help="Grid height (number of rows)"
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=1.0,
        help="Distance between adjacent nodes (default: 1.0)",
    )
    parser.add_argument(
        "--obstacles",
        type=str,
        default=None,
        help='Space-separated list of obstacle coordinates as "x,y" pairs, '
        'e.g. "3,3 4,3 3,4"',
    )
    parser.add_argument(
        "--map-id",
        type=str,
        default="basic_grid",
        help="Map ID to embed in map_info (default: basic_grid)",
    )
    parser.add_argument(
        "--map-version",
        type=str,
        default="1.0",
        help="Map version string to embed in map_info (default: 1.0)",
    )
    parser.add_argument(
        "--allowed-deviation-xy",
        type=float,
        default=0.5,
        help="Allowed XY deviation per node (default: 0.5)",
    )
    parser.add_argument(
        "--allowed-deviation-theta",
        type=float,
        default=0.1,
        help="Allowed theta deviation per node (default: 0.1)",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=1.0,
        help="Max speed per edge (default: 1.0)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output JSON file path (default: <map-id>.json)",
    )

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        obstacles = parse_obstacles(args.obstacles)

        grid = generate(
            width=args.width,
            height=args.height,
            obstacles=obstacles,
            spacing=args.spacing,
            map_id=args.map_id,
            map_version=args.map_version,
            allowed_deviation_xy=args.allowed_deviation_xy,
            allowed_deviation_theta=args.allowed_deviation_theta,
            max_speed=args.max_speed,
        )
    except (ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))
        return 2

    output_file = args.output or Path(f"{args.map_id}.json")

    with output_file.open("w") as f:
        json.dump(grid, f, indent=2, ensure_ascii=False)

    print(
        f"Generated {len(grid['nodes'])} nodes and "
        f"{len(grid['edges'])} edges -> {output_file}"
    )

    return 0


if __name__ == "__main__":
    sys.exit(main())
