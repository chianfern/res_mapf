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


class MapfSolverError(Exception):
    """Base exception for all solver-related errors."""

    pass


class NoSolutionFound(MapfSolverError):
    """Raised when the solver could not find a valid solution."""


class UnknownWaypointError(MapfSolverError):
    """Raised when an agent's start/goal references a location not present in the map."""

    def __init__(self, location: str, agent_id: str | None = None):
        self.location = location
        self.agent_id = agent_id
        msg = f"Unknown waypoint '{location}'"
        if agent_id is not None:
            msg += f" (referenced by agent '{agent_id}')"
        super().__init__(msg)


class SolverTimeoutError(MapfSolverError):
    """Raised when solver exceeds time limits."""

    pass


class InvalidMapError(MapfSolverError):
    """Raised when the map/layout passed to the solver is structurally invalid
    (e.g. disconnected graph, no nodes, duplicate/ambiguous coordinates)."""
