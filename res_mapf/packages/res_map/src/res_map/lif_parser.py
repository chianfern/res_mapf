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

import json
import logging
from pathlib import Path

from .map_data import Edge, MapData

logger = logging.getLogger(__name__)


def load_lif(path: str | Path) -> MapData:
    """
    Load a single-layout LIF file.
    TODO: Support multiple layouts

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    ValueError
        If the LIF is malformed, contains zero or multiple layouts, or an
        edge references an unknown node.
    """
    path = Path(path)

    if not path.is_file():
        raise FileNotFoundError(f"LIF file not found: {path}")

    logger.info("Loading LIF file '%s'.", path)

    lif = _load_json(path)
    layouts = _get_layouts(lif, path)

    if len(layouts) != 1:
        raise ValueError(f"{path}: expected exactly one layout, found {len(layouts)}.")

    return _parse_layout(layouts[0], path)


def _load_json(path: Path) -> dict:
    """Load and validate a JSON file."""

    try:
        with path.open("r", encoding="utf-8") as fp:
            data: dict = json.load(fp)
            return data
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON.") from exc


def _get_layouts(lif: dict, path: Path) -> list[dict]:
    """Return all layouts from a LIF file."""

    layouts: list[dict] | None = lif.get("layouts")

    if not layouts:
        raise ValueError(f"{path}: missing or empty 'layouts'.")

    return layouts


def _parse_layout(layout: dict, path: Path) -> MapData:
    """Parse a single LIF layout."""

    layout_id = layout.get("layoutId", "")

    logger.info("Parsing layout '%s'.", layout_id or "<unknown>")

    world_positions = _parse_nodes(layout, path)
    edges = _parse_edges(layout, world_positions, path)

    logger.info(
        "Parsed layout '%s' (%d nodes, %d edges).",
        layout_id,
        len(world_positions),
        len(edges),
    )

    return MapData(
        world_positions=world_positions,
        world_position_to_name={pos: node for node, pos in world_positions.items()},
        edges=edges,
        map_name=layout_id,
    )


def _parse_nodes(
    layout: dict,
    path: Path,
) -> dict[str, tuple[float, float]]:
    """Parse nodes from a layout."""

    nodes = layout.get("nodes", [])

    if not nodes:
        raise ValueError(f"{path}: layout contains no nodes.")

    world_positions: dict[str, tuple[float, float]] = {}

    for node in nodes:
        try:
            node_id = node["nodeId"]

            if node_id in world_positions:
                raise ValueError(f"{path}: duplicate node ID '{node_id}'.")

            position = node["nodePosition"]

            world_positions[node_id] = (
                float(position["x"]),
                float(position["y"]),
            )

        except KeyError as exc:
            raise ValueError(
                f"{path}: node missing required field {exc.args[0]!r}."
            ) from exc

    logger.debug("Parsed %d nodes.", len(world_positions))

    return world_positions


def _parse_edges(
    layout: dict,
    world_positions: dict[str, tuple[float, float]],
    path: Path,
) -> list[Edge]:
    """Parse edges from a layout."""

    raw_edges = layout.get("edges", [])
    edges: list[Edge] = []

    if not raw_edges:
        logger.warning(
            "Layout '%s' contains no edges.",
            layout.get("layoutId", "<unknown>"),
        )

    for index, raw_edge in enumerate(raw_edges):
        try:
            edge_id = raw_edge.get("edgeId", f"edge-{index}")
            start = raw_edge["startNodeId"]
            end = raw_edge["endNodeId"]

        except KeyError as exc:
            raise ValueError(
                f"{path}: edge missing required field {exc.args[0]!r}."
            ) from exc

        for node in (start, end):
            if node not in world_positions:
                raise ValueError(
                    f"{path}: edge '{edge_id}' references unknown node '{node}'."
                )

        edges.append(Edge(node_a=start, node_b=end))

    logger.debug("Parsed %d edges.", len(edges))

    return edges
