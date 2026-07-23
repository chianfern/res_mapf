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
Utilities for estimating grids.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from res_map.map_data import MapData


@dataclass
class GridMap:
    """
    Integer grid representation of a map with named nodes.

    Attributes:
        grid_nodes:     Mapping from node name to (x, y) integer grid
                        coordinates. x increases left to right, y increases
                        top to bottom (row 0 = largest y in LIF).
        dimension:      [width, height] of the grid.
        obstacles:      Nodes that are obstacles.
        map_name:       The map (LIF layoutId) this grid represents.
    """

    grid_nodes: Dict[str, Tuple[int, int]]
    dimension: List[int]
    obstacles: List[Tuple[int, int]] = field(default_factory=list)
    map_name: str = ""


def snap_to_grid(map_data: MapData) -> GridMap:
    """
    Attempts to convert an axis-aligned map to a grid.

    Assumes the map's rows and columns are parallel to the
    x and y axes respectively, with uniform spacing in both directions.

    Grid cell size is inferred as the greatest common divisor of all non-zero coordinate gaps
    between nodes. The origin is shifted so the minimum x maps to column 0
    and the maximum y maps to row 0 (row 0 = top of map).

    This function does NOT determine obstacles. Call infer_obstacles()
    separately, or implement your own obstacle logic.

    Args:
        map_data: MapData loaded from a LIF file.

    Returns:
        GridMap with integer grid coordinates and dimensions.

    Raises:
        ValueError: if world positions don't form a detectable regular grid,
                    or if two nodes snap to the same cell.
    """
    positions = list(map_data.world_positions.values())
    spacing = 1.0 if len(positions) == 1 else _infer_spacing(positions)

    min_x = min(x for x, _ in positions)
    raw_max_y = max(y for _, y in positions)

    grid_nodes: Dict[str, Tuple[int, int]] = {}
    for node_id, (x, y) in map_data.world_positions.items():
        gx = round((x - min_x) / spacing)
        gy = round((raw_max_y - y) / spacing)
        grid_nodes[node_id] = (gx, gy)

    # Check for collisions after snapping.
    seen: Dict[Tuple[int, int], str] = {}
    for node_id, cell in grid_nodes.items():
        if cell in seen:
            raise ValueError(
                f"Nodes '{seen[cell]}' and '{node_id}' both snap to grid cell "
                f"{cell} (inferred spacing = {spacing} m). "
                f"Nodes may not lie on a regular grid."
            )
        seen[cell] = node_id

    max_x = max(gx for gx, _ in grid_nodes.values())
    max_y = max(gy for _, gy in grid_nodes.values())

    return GridMap(
        grid_nodes=grid_nodes,
        dimension=[max_x + 1, max_y + 1],
        map_name=map_data.map_name,
    )


def infer_obstacles(
    map_data: MapData,
    grid_map: GridMap,
    extra_obstacles: Optional[Set[str]] = None,
) -> List[Tuple[int, int]]:
    """
    Approximately determines grid obstacles from node connectivity.
    This is provided as a sample.

    Nodes with 0 or 1 edges are considered obstacles
    NOTE: nodes with 2 or more edges are determined as fully passable in the grid.

    Args:
        map_data (MapData): MapData loaded from a LIF file.
        grid_map (GridMap): GridMap produced by snap_to_grid().
        extra_obstacles (Optional[Set[str]], optional): Optional node IDs to mark as obstacles regardless
                         of edge count.

    Returns:
        List of (x, y) integer grid cells that are impassable.

    Raises:
        ValueError: if extra_obstacles references unknown node IDs.
    """
    if extra_obstacles is None:
        extra_obstacles = set()

    unknown = extra_obstacles - set(map_data.world_positions.keys())
    if unknown:
        raise ValueError(
            f"extra_obstacles references unknown node IDs: {sorted(unknown)}"
        )

    edge_count: Dict[str, int] = {nid: 0 for nid in map_data.world_positions}
    for edge in map_data.edges:
        edge_count[edge.node_a] += 1
        edge_count[edge.node_b] += 1

    auto_obstacles: Set[str] = {nid for nid, count in edge_count.items() if count < 2}

    all_obstacles: Set[str] = auto_obstacles | extra_obstacles

    return [grid_map.grid_nodes[nid] for nid in all_obstacles]


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _infer_spacing(positions: List[Tuple[float, float]]) -> float:
    """Infer grid cell size as  of all non-zero coordinate gaps.
    Assumes the map is axis-aligned - rows and columns are parallel to the
    x and y axes respectively, with uniform spacing in both directions.
    """
    xs = sorted({x for x, _ in positions})
    ys = sorted({y for _, y in positions})

    gaps = []
    for coords in (xs, ys):
        for a, b in zip(coords, coords[1:]):
            gap = abs(b - a)
            if gap > 1e-9:
                gaps.append(gap)

    if not gaps:
        raise ValueError(
            "Cannot infer grid spacing: all nodes share the same coordinate "
            "on at least one axis."
        )

    spacing = gaps[0]
    for g in gaps[1:]:
        spacing = _float_gcd(spacing, g)

    return spacing


def _float_gcd(a: float, b: float, tolerance: float = 1e-6) -> float:
    """Euclidean algorithm to find greatest common divisor for floats, stopping when remainder < tolerance."""
    while b > tolerance:
        a, b = b, a % b
    return a
