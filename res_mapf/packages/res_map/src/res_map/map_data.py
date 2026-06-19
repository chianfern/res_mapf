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

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class Edge:
    """
    A connection between two nodes in the map.
    """

    node_a: str  # ID of the first node.
    node_b: str  # ID of the second node.
    bidirectional: bool  # If True, robots may travel in both directions. If False, travel is only permitted from node_a to node_b.


@dataclass(frozen=True)
class MapData:
    """
    Domain model of a map loaded from a LIF JSON file.
    """

    world_positions: Dict[
        str, Tuple[float, float]
    ]  # Mapping from node name to (x, y) real-world coordinates in metres.
    world_position_to_name: Dict[
        Tuple[float, float], str
    ]  # Mapping from coordinates to name
    edges: List[Edge]  # Connections between nodes


def load_map_data(lif_path: str) -> MapData:
    """
    Parse a LIF JSON file.

    Args:
        lif_path: Path to the LIF JSON file.

    Returns:
        MapData: Parsed map data.

    Raises:
        FileNotFoundError: if the file does not exist.
        ValueError: if the file is malformed or an edge references an
                    unknown node.
    """
    p = Path(lif_path)
    if not p.exists():
        raise FileNotFoundError(f"LIF file not found: {lif_path}")

    with p.open(encoding="utf-8") as f:
        lif = json.load(f)

    # --- Parse nodes ---
    world_positions: Dict[str, Tuple[float, float]] = {}
    for node in lif.get("nodes", []):
        node_id = node["node_id"]
        world_positions[node_id] = (float(node["x"]), float(node["y"]))
    world_position_to_name = {v: k for k, v in world_positions.items()}

    if not world_positions:
        raise ValueError(f"{lif_path}: no nodes found.")

    # --- Parse edges ---
    edges: List[Edge] = []
    for idx, raw_edge in enumerate(lif.get("edges", [])):
        node_a = raw_edge["start_node_id"]
        node_b = raw_edge["end_node_id"]

        # Validate both endpoints exist.
        for node_id in (node_a, node_b):
            if node_id not in world_positions:
                raise ValueError(
                    f"{lif_path}: edge '{raw_edge.get('edge_id', idx)}' "
                    f"references unknown node '{node_id}'."
                )

        edges.append(
            Edge(
                node_a=node_a,
                node_b=node_b,
                bidirectional=bool(raw_edge.get("bidirectional", True)),
            )
        )

    return MapData(
        world_positions=world_positions,
        world_position_to_name=world_position_to_name,
        edges=edges,
    )
