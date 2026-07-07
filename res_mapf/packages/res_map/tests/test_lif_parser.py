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
from pathlib import Path
from typing import Any

import pytest

from res_map.lif_parser import load_lif
from res_map.map_data import Edge, MapData

FIXTURES_DIR = Path(__file__).parent / "fixtures"
SAMPLE_LIF = FIXTURES_DIR / "sample_lif.json"


def _write_lif(tmp_path: Path, data: dict, name: str = "test.lif.json") -> Path:
    """Write a dict to a JSON file in tmp_path and return its path."""
    path = tmp_path / name
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _minimal_layout(**overrides: Any) -> dict:
    """A minimal valid single-layout LIF document, with optional overrides."""
    layout = {
        "layoutId": "layout-1",
        "layoutName": "Test Layout",
        "nodes": [
            {"nodeId": "A", "nodePosition": {"x": 0.0, "y": 0.0}},
            {"nodeId": "B", "nodePosition": {"x": 1.5, "y": 2.5}},
        ],
        "edges": [
            {"edgeId": "e1", "startNodeId": "A", "endNodeId": "B"},
        ],
    }
    layout.update(overrides)
    return {"layouts": [layout]}


@pytest.mark.skipif(
    not SAMPLE_LIF.is_file(),
    reason="tests/fixtures/sample.lif.json not present",
)
def test_load_lif_sample_file_parses() -> None:
    map_data = load_lif(SAMPLE_LIF)
    assert isinstance(map_data, MapData)
    assert len(map_data.world_positions) > 0
    assert len(map_data.edges) > 0
    for edge in map_data.edges:
        assert edge.node_a in map_data.world_positions
        assert edge.node_b in map_data.world_positions
    for node_id, pos in map_data.world_positions.items():
        assert map_data.world_position_to_name[pos] == node_id


def test_load_lif_minimal_valid(tmp_path: Path) -> None:
    path = _write_lif(tmp_path, _minimal_layout())
    map_data = load_lif(path)
    assert map_data.world_positions == {"A": (0.0, 0.0), "B": (1.5, 2.5)}
    assert map_data.edges == [Edge(node_a="A", node_b="B")]


def test_load_lif_file_not_found(tmp_path: Path) -> None:
    missing = tmp_path / "does_not_exist.lif.json"
    with pytest.raises(FileNotFoundError, match=str(missing)):
        load_lif(missing)


def test_load_lif_invalid_json(tmp_path: Path) -> None:
    path = tmp_path / "bad.json"
    path.write_text("{not valid json", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid JSON"):
        load_lif(path)


def test_load_lif_edge_references_unknown_node(tmp_path: Path) -> None:
    layout = _minimal_layout(
        edges=[{"edgeId": "e1", "startNodeId": "A", "endNodeId": "Z"}]
    )
    path = _write_lif(tmp_path, layout)
    with pytest.raises(ValueError, match="unknown node"):
        load_lif(path)
