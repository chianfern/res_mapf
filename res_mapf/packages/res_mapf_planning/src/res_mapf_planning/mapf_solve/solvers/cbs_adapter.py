# Copyright (C) 2025-2026 ROS-Industrial Consortium Asia Pacific
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

from typing import Dict, List, Sequence, Tuple, TypedDict


from res_map.map_data import MapData
from res_map.grid.grid_utils import GridMap, infer_obstacles, snap_to_grid
from res_mapf_planning.cbs.cbs import CBS, AgentContext, CBSPlan, Environment
from res_mapf_planning.mapf_solve.exceptions import NoSolutionError
from res_mapf_planning.mapf_solve.mapf_solver_base import (
    Location,
    MAPFAgent,
    MAPFSolverBase,
    Obstacle,
)
from res_mapf_planning.mapf_solve.models.models import SolverLocation, SolverPlan, Step


class _CBSOutput(TypedDict):
    schedule: CBSPlan
    cost: int


class CBSAdapter(MAPFSolverBase):
    """Adapter to the cbs solver for local testing.

    Named locations (e.g. "P01") are resolved to grid coordinates via the
    grid map data.
    """

    def __init__(self, grid_map: GridMap) -> None:
        self._grid_map: GridMap = grid_map

    def solve(
        self,
        items: Sequence[MAPFAgent],
        input_obstacles: Sequence[Obstacle] = [],
    ) -> Sequence[SolverPlan]:
        task_ids = {}  # Track task IDs
        agents: List[AgentContext] = []

        for item in items:
            agent: AgentContext = {
                "start": list(self._to_cbs_coords(item.start)),
                "goal": list(self._to_cbs_coords(item.goal)),
                "name": item.agent_id,
            }
            agents.append(agent)
            task_ids[item.agent_id] = item.task_id if item.task_id else ""

        obstacles: List[Tuple[int, int]] = self._grid_map.obstacles
        for obs in input_obstacles:
            obstacles.append(self._to_cbs_coords(obs.location))

        print("Agents:", agents)
        print("Number of input obstacles:", len(input_obstacles), flush=True)
        for input_obstacle in input_obstacles:
            print("input obstacle:", input_obstacle, flush=True)

        print("Solving with CBS..", flush=True)
        output = self._solve_cbs(self._grid_map.dimension, agents, obstacles)
        plans = self._convert_cbs(output, task_ids)
        print("Solved.", flush=True)

        return plans

    def _to_cbs_coords(self, location: Location) -> Tuple[int, int]:
        """Resolve a Location to an (x, y) integer grid coordinate tuple."""
        if location.is_named():
            if location.name in self._grid_map.grid_nodes:
                return self._grid_map.grid_nodes[location.name]
            raise ValueError(f"Named location '{location.name}' not found in map.")
        if location.is_coordinates():
            return (int(location.x), int(location.y))
        raise ValueError(f"Invalid Location: {location}")

    def _solve_cbs(
        self,
        dimension: Sequence[int],
        agents: Sequence[AgentContext],
        obstacles: Sequence[Tuple[int, int]],
    ) -> _CBSOutput:
        env = Environment(dimension, agents, obstacles)
        cbs = CBS(env)
        solution = cbs.search()
        if not solution:
            raise NoSolutionError("CBS could not find a valid solution")

        # Write to output file
        output: _CBSOutput = {
            "schedule": solution,
            "cost": env.compute_solution_cost(solution),
        }
        # with open("cbs_output", "w") as output_yaml:
        #     yaml.safe_dump(output, output_yaml)

        return output

    def _convert_cbs(
        self, cbs_output: _CBSOutput, task_ids: Dict[str, str]
    ) -> Sequence[SolverPlan]:
        """
        Convert output from cbs.py.

        Args:
            cbs_output (dict): Object loaded from cbs.py's output.yaml
        """

        coord_to_node: Dict[Tuple[int, int], str] = {
            v: k for k, v in self._grid_map.grid_nodes.items()
        }

        plans = []
        for agent_name, agent_path in cbs_output["schedule"].items():
            steps = []

            # Create steps.

            for idx in range(
                len(agent_path) - 1
            ):  # Exclude final waypoint from the loop.
                waypoint = agent_path[idx]
                next_waypoint = agent_path[idx + 1]

                timestep = waypoint["t"]
                step_from = SolverLocation(
                    node=coord_to_node[(waypoint["x"], waypoint["y"])],
                    x=waypoint["x"],
                    y=waypoint["y"],
                )
                step_to = SolverLocation(
                    node=coord_to_node[(next_waypoint["x"], next_waypoint["y"])],
                    x=next_waypoint["x"],
                    y=next_waypoint["y"],
                )

                steps.append(
                    Step(
                        timestep=timestep,
                        step_from=step_from,
                        step_to=step_to,
                        task_id=task_ids[agent_name],
                    )
                )
            plans.append(SolverPlan(agent_name=agent_name, steps=steps))

        return plans
