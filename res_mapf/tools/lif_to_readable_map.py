#!/usr/bin/env python3

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
Generates a human-friendly grid map displaying obstacles from a LIF JSON file.

Usage
-----
    python lif_to_readable_map.py warehouse.lif.json warehouse_grid_map.csv --obstacles P5 P12

Input format (from to_lif.py): see warehouse.lif.json

Output format
-------------
A grid CSV where each cell is a zero-padded node name or X (obstacle)

Obstacles are determined:

1. Automatically by edge-connectivity:
   Classical MAPF solvers (CBS, A*) assume an undirected grid.
   Currently assume
     - 0 or 1 edges: node is unreachable
     - 2+ edges: node is passable
   NOTE: nodes with 2+ edges but missing specific cross-links are still treated
   as fully passable. The solver may plan transitions not present in the LIF.

2. Passed manually with --obstacles:
   Pass additional node_ids to set as obstacles
"""

import argparse
import sys
from pathlib import Path

import json

from res_map.lif_parser import load_lif
from res_map.grid.grid_utils import infer_obstacles, snap_to_grid


def convert(
    lif_path: Path,
    output_path: Path,
    obstacle_node_ids: set[str],
) -> None:
    # --- Load and parse LIF ---
    map_data = load_lif(lif_path)
    with lif_path.open(encoding="utf-8") as f:
        lif_raw = json.load(f)
    layouts = lif_raw.get("layouts", [])
    map_id = layouts[0].get("layoutId", lif_path.stem) if layouts else lif_path.stem

    # --- Validate explicit obstacle node_ids ---
    unknown = obstacle_node_ids - set(map_data.world_positions.keys())
    if unknown:
        sys.exit(
            f"Error: --obstacles node_id(s) not found in map '{map_id}': "
            f"{sorted(unknown)}\n"
            f"Available node_ids: {sorted(map_data.world_positions.keys())}"
        )

    # --- Snap to grid and derive obstacles ---
    grid = snap_to_grid(map_data)
    obstacles = set(infer_obstacles(map_data, grid, extra_obstacles=obstacle_node_ids))

    # --- Determine which node_ids are obstacles for the summary. ---
    obstacle_node_set = {
        nid for nid, cell in grid.grid_nodes.items() if cell in obstacles
    }
    auto_obstacles = obstacle_node_set - obstacle_node_ids
    all_node_ids = set(map_data.world_positions.keys())

    # --- Names padded to uniform width for column alignment in the CSV. ---
    max_len = max(len(nid) for nid in all_node_ids)

    def padded_name(node_id: str) -> str:
        return node_id.ljust(max_len)

    node_id_to_name: dict[str, str] = {nid: padded_name(nid) for nid in all_node_ids}
    cell_width = max_len

    # --- Build grid lookup ---
    max_x, max_y = grid.dimension[0] - 1, grid.dimension[1] - 1
    cell_to_node: dict[tuple[int, int], str] = {
        v: k for k, v in grid.grid_nodes.items()
    }

    # --- Write output CSV ---
    output_path.parent.mkdir(parents=True, exist_ok=True)

    min_x = min(x for x, _ in map_data.world_positions.values())
    raw_max_y = max(y for _, y in map_data.world_positions.values())
    raw_min_y = min(y for _, y in map_data.world_positions.values())
    spacing = (raw_max_y - raw_min_y) / max_y if max_y > 0 else 1.0

    with output_path.open("w", encoding="utf-8") as f:
        f.write(f"# RES map — generated from {lif_path.name}\n")
        f.write(f"# Map: {map_id}\n")
        f.write(
            f"# Grid: {grid.dimension[0]} x {grid.dimension[1]}  "
            f"(spacing {spacing:.6g} m/cell)\n"
            f"# LIF origin: ({min_x:.6g}, {raw_max_y:.6g}) m  "
            f"# x increases left to right, y increases bottom to top in LIF\n"
            f"# CSV row 0 = top of map, last row = bottom of map\n"
        )
        f.write("\n")

        for gy in range(max_y + 1):
            row_cells = []
            for gx in range(max_x + 1):
                nid = cell_to_node.get((gx, gy))
                if nid is None or (gx, gy) in obstacles:
                    cell = "X".ljust(cell_width)
                else:
                    cell = node_id_to_name[nid]
                row_cells.append(cell)
            f.write(",".join(row_cells) + "\n")

    # --- Summary ---
    n_gaps = grid.dimension[0] * grid.dimension[1] - len(all_node_ids)
    print(f"Wrote {output_path}")
    print(f"  Map:                {map_id}")
    print(f"  Grid:               {grid.dimension[0]} x {grid.dimension[1]}")
    print(f"  Nodes:              {len(all_node_ids)}")
    print(f"  Auto obstacles:     {len(auto_obstacles)} (0 or 1 edge)")
    print(f"  Explicit obstacles: {len(obstacle_node_ids)}")
    print(f"  Gaps:               {n_gaps}")
    if auto_obstacles:
        print(
            f"  Auto obstacle names: {sorted(node_id_to_name[n] for n in auto_obstacles)}"
        )
    if obstacle_node_ids:
        print(
            f"  Explicit obstacle names: {sorted(node_id_to_name[n] for n in obstacle_node_ids)}"
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("lif_file", type=Path, help="Input .lif.json file.")
    parser.add_argument("output_file", type=Path, help="Output .csv file to write.")
    parser.add_argument(
        "--obstacles",
        nargs="*",
        default=[],
        metavar="NODE_ID",
        help=(
            "node_id(s) to mark as static obstacles, e.g. --obstacles P5 P12. "
            "Use the node_id strings from the LIF file."
        ),
    )
    args = parser.parse_args()
    convert(
        lif_path=args.lif_file,
        output_path=args.output_file,
        obstacle_node_ids=set(args.obstacles),
    )


if __name__ == "__main__":
    main()
