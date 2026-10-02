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

import uuid

from res_mapf_planning.traffic_dependencies.models.plan import Plan, PlanId, Waypoint
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_plan_execution.plan_execution.dependency_manager import DependencyManager



def _uuid_from_str(name, namespace=uuid.UUID("00000000-0000-0000-0000-000000000000")) -> uuid.UUID:
    return uuid.uuid5(namespace, name)

def _make_alice_plan() -> Plan:
    return Plan(
        plan_id=PlanId(_uuid_from_str("alice"), plan_version=0),
        waypoints=[
            Waypoint(name="A", position=(0.0, 0.0), progress=0.0),
            Waypoint(name="B", position=(1.0, 0.0), progress=1.0),
            Waypoint(name="C", position=(2.0, 0.0), progress=2.0),
            Waypoint(name="D", position=(3.0, 0.0), progress=3.0),
            Waypoint(name="E", position=(4.0, 0.0), progress=4.0),
            Waypoint(name="F", position=(5.0, 0.0), progress=5.0),
            Waypoint(name="G", position=(6.0, 0.0), progress=6.0),
        ],
    )

def _make_bob_plan() -> Plan:
    # Alice must complete waypoint B before Bob may move towards waypoint Y

    traffic_dependency = TrafficDependency("alice", _uuid_from_str("alice"), 1.0)
    departure_blockers = [traffic_dependency]


    return Plan(
        plan_id=PlanId(destination_session=_uuid_from_str("bob"), plan_version=0),
        waypoints=[

            Waypoint(name="V", position=(0.0, 0.0), progress=0.0),
            Waypoint(name="W", position=(1.0, 0.0), progress=1.0),
            Waypoint(name="X", position=(2.0, 0.0), progress=2.0),
            Waypoint(name="Y", position=(3.0, 0.0), progress=3.0, departure_blockers=departure_blockers),
            Waypoint(name="Z", position=(4.0, 0.0), progress=4.0),
        ],
    )

def test_dm_flow():
    """
    max_enqueued = 2
    """
    dm = DependencyManager()

    alice_plan = _make_alice_plan()
    dm.set_plan("alice", alice_plan)
    print("alice planid:", dm._robots["alice"].plan.plan_id, dm._robots["alice"].plan.plan_id.plan_version)

    # dm.set_plan("bob", _make_bob_plan())

    assert dm.get_valid_waypoints_and_advance("alice") == [(1, Waypoint(name='B', position=(1.0, 0.0), progress=1.0, departure_blockers=[], departure_action='')), (2, Waypoint(name='C', position=(2.0, 0.0), progress=2.0, departure_blockers=[], departure_action=''))]
    dm.update_progress("alice", 1)

    assert dm.get_valid_waypoints_and_advance("alice") == [(3, Waypoint(name='D', position=(3.0, 0.0), progress=3.0, departure_blockers=[], departure_action=''))]
    dm.update_progress("alice", 2)
    dm.update_progress("alice", 3)

    dm.compute_commit_cut()

    print("cut index:", dm._robots["alice"].cut_index)
    print("cut waypoint:", dm._robots["alice"].plan.waypoints[dm._robots["alice"].cut_index])

    dm.update_progress("alice", 4)

    new_plan = Plan(
        plan_id=PlanId(_uuid_from_str("alice"), plan_version=1),
        waypoints=[
            Waypoint(name="H", position=(0.0, 0.0), progress=0.0),
            Waypoint(name="I", position=(1.0, 0.0), progress=1.0),
            Waypoint(name="J", position=(2.0, 0.0), progress=2.0),
            Waypoint(name="K", position=(3.0, 0.0), progress=3.0),
            Waypoint(name="L", position=(4.0, 0.0), progress=4.0),
        ],
    )
    dm.update_plan_after_cut("alice", new_plan)
    print("after updating plan:", dm._robots["alice"].plan.waypoints)
    print("new planid:", dm._robots["alice"].plan.plan_id, dm._robots["alice"].plan.plan_id.plan_version)



def test_compute_commit_cut_after_failed_plan():
    """
    on_plan_failed clears a robot's plan to None but keeps the robot in
    the dependency manager so other robots stay blocked on it until a
    replan arrives. compute_commit_cut must not crash while iterating
    that robot's own (now-None) plan.
    """
    dm = DependencyManager()
    dm.set_plan("alice", _make_alice_plan())

    dm.on_plan_failed("alice")

    result = dm.compute_commit_cut()

    assert "alice" not in result.committed_locations, (
        "A robot with no plan cannot have a committed location"
    )


def test_compute_commit_cut_blocker_referencing_failed_plan():
    """
    A departure blocker can reference a robot whose plan has since failed:
    on_plan_failed clears that robot's plan to None without marking its
    plan_id complete, so other robots must remain blocked on it.
    Resolving that blocker must not crash on the None plan.
    """
    dm = DependencyManager()

    alice_plan = _make_alice_plan()
    dm.set_plan("alice", alice_plan)

    blocker = TrafficDependency(
        name="alice",
        plan_id=alice_plan.plan_id,
        required_progress=1.0,
    )
    bob_plan = Plan(
        plan_id=PlanId(destination_session=uuid.uuid4(), plan_version=0),
        waypoints=[
            Waypoint(
                name="X", position=(0.0, 1.0), progress=0.0, departure_blockers=[blocker]
            ),
            Waypoint(name="Y", position=(1.0, 1.0), progress=1.0),
        ],
    )
    dm.set_plan("bob", bob_plan)

    dm.on_plan_failed("alice")

    dm.compute_commit_cut()
