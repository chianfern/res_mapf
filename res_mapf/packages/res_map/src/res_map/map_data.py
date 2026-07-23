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

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Dict, List, Tuple

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Edge:
    """
    A connection between two nodes in the map.
    """

    node_a: str  # ID of the start node
    node_b: str  # ID of the end node
    # Edges are directional (node_a -> node_b), matching the LIF spec.
    # A layout representing two-way travel will contain two Edge entries,
    # one in each direction.


@dataclass(frozen=True)
class MapData:
    """
    Domain model of a map loaded from a LIF JSON file.
    Use `lif_parser.load_lif()` to construct this from a LIF file.
    """

    world_positions: Dict[
        str, Tuple[float, float]
    ]  # Mapping from node name to (x, y) real-world coordinates in metres.
    world_position_to_name: Dict[
        Tuple[float, float], str
    ]  # Mapping from coordinates to name
    edges: List[Edge]  # Connections between nodes
    map_name: str = ""  # LIF layoutId.
