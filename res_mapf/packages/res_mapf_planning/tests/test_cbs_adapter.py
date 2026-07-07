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

import pytest
from res_map.grid.grid_utils import GridMap
from res_mapf_planning.mapf_solve.mapf_solver_base import Location, MAPFAgent
from res_mapf_planning.mapf_solve.models.models import SolverPlan
from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter


@pytest.fixture
def grid_map() -> GridMap:
    dimension = [5, 5]

    # CBS searches every cell in the grid, not just named nodes, so a
    # solution path may pass through unnamed cells. Name every cell so the
    # adapter can always resolve a name for a coordinate.
    grid_nodes = {
        f"_cell_{x}_{y}": (x, y)
        for x in range(dimension[0])
        for y in range(dimension[1])
    }

    return GridMap(
        grid_nodes=grid_nodes,
        dimension=dimension,
    )


@pytest.fixture
def cbs_adapter(grid_map: GridMap) -> CBSAdapter:
    return CBSAdapter(grid_map)


class TestCBSAdapter:
    def test_solve(self, cbs_adapter: CBSAdapter) -> None:
        agents = [
            MAPFAgent(
                task_id="test0",
                agent_id="0",
                start=Location(name="_cell_0_0"),
                goal=Location(name="_cell_2_0"),
            ),
            MAPFAgent(
                task_id="test1",
                agent_id="1",
                start=Location(name="_cell_2_0"),
                goal=Location(name="_cell_0_0"),
            ),
            MAPFAgent(
                task_id="test2",
                agent_id="2",
                start=Location(name="_cell_1_0"),
                goal=Location(name="_cell_1_2"),
            ),
            MAPFAgent(
                task_id="test3",
                agent_id="3",
                start=Location(name="_cell_4_0"),
                goal=Location(name="_cell_4_4"),
            ),
        ]

        plans = cbs_adapter.solve(agents, obstacles=[])

        assert all(isinstance(plan, SolverPlan) for plan in plans)
