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
Generate a grid MapData and export it to a LIF file.


Example:
python3 generate_lif_from_grid.py --width 7 --height 7 --spacing 1.0 --map-id basic_grid -o basic_grid.json

"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from res_map.map_data import Edge, MapData


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


def generate_grid_map_data(
    width: int,
    height: int,
    obstacles: set[tuple[int, int]] | None = None,
    spacing: float = 1.0,
) -> MapData:
    """
    Build a MapData representing a 4-connected grid map.

    node: P_3_4
    edge: E_3_4_to_4_4
    Raises
    ------
    ValueError
        If width/height are not positive, or an obstacle lies outside
        the grid bounds.
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

    world_positions: dict[str, tuple[float, float]] = {}
    node_lookup: dict[tuple[int, int], str] = {}

    for y in range(height):
        for x in range(width):
            if (x, y) in obstacles:
                continue

            node_id = f"P_{x}_{y}"
            node_lookup[(x, y)] = node_id
            world_positions[node_id] = (float(x * spacing), float(y * spacing))

    edges: list[Edge] = []
    directions = [(1, 0), (0, 1)]  # right, down; reverse edge added separately

    for y in range(height):
        for x in range(width):
            if (x, y) not in node_lookup:
                continue

            start_node_id = node_lookup[(x, y)]

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if (nx, ny) not in node_lookup:
                    continue

                end_node_id = node_lookup[(nx, ny)]
                edges.append(Edge(node_a=start_node_id, node_b=end_node_id))
                edges.append(Edge(node_a=end_node_id, node_b=start_node_id))

    return MapData(
        world_positions=world_positions,
        world_position_to_name={pos: node for node, pos in world_positions.items()},
        edges=edges,
    )


def export_lif(
    map_data: MapData,
    *,
    layout_id: str = "basic_grid",
    layout_name: str = "Basic Grid",
) -> dict:
    """
    Convert a MapData into a single-layout LIF document, matching the
    schema read by res_map.lif_parser.load_lif (nodeId/nodePosition,
    edgeId/startNodeId/endNodeId, wrapped in a "layouts" list).
    """
    nodes = [
        {
            "nodeId": node_id,
            "nodePosition": {"x": x, "y": y},
        }
        for node_id, (x, y) in map_data.world_positions.items()
    ]

    edges = [
        {
            "edgeId": f"E_{index}_{edge.node_a}_to_{edge.node_b}",
            "startNodeId": edge.node_a,
            "endNodeId": edge.node_b,
        }
        for index, edge in enumerate(map_data.edges)
    ]

    return {
        "layouts": [
            {
                "layoutId": layout_id,
                "layoutName": layout_name,
                "nodes": nodes,
                "edges": edges,
            }
        ]
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a grid MapData and export it to a LIF file.",
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
        help="Distance between adjacent nodes, in metres (default: 1.0)",
    )
    parser.add_argument(
        "--obstacles",
        type=str,
        default=None,
        help='Space-separated list of obstacle coordinates as "x,y" pairs, '
        'e.g. "3,3 4,3 3,4"',
    )
    parser.add_argument(
        "--layout-id",
        type=str,
        default="basic_grid",
        help="LIF layoutId to embed (default: basic_grid)",
    )
    parser.add_argument(
        "--layout-name",
        type=str,
        default="Basic Grid",
        help="LIF layoutName to embed (default: 'Basic Grid')",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output LIF JSON file path (default: <layout-id>.lif.json)",
    )

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        obstacles = parse_obstacles(args.obstacles)
        map_data = generate_grid_map_data(
            width=args.width,
            height=args.height,
            spacing=args.spacing,
            obstacles=obstacles,
        )
    except (ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))
        return 2

    lif = export_lif(
        map_data,
        layout_id=args.layout_id,
        layout_name=args.layout_name,
    )

    output_file = args.output or Path(f"{args.layout_id}.lif.json")

    with output_file.open("w", encoding="utf-8") as f:
        json.dump(lif, f, indent=2, ensure_ascii=False)

    print(
        f"Generated {len(map_data.world_positions)} nodes and "
        f"{len(map_data.edges)} edges -> {output_file}"
    )

    return 0


if __name__ == "__main__":
    sys.exit(main())
