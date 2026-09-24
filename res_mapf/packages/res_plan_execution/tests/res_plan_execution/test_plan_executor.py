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

from typing import Callable
from uuid import uuid4

from res_mapf_planning.traffic_dependencies.models.plan import Plan, PlanId, Waypoint
from res_plan_execution.plan_execution.dependency_manager import DependencyManager
from res_plan_execution.plan_execution.plan_executor import PlanExecutor
from res_plan_execution.plan_execution.transport.executor_base_transport import (
    ExecutorBaseTransport,
)
from res_plan_execution.robot_controllers.base_robot_controller import (
    BaseRobotController,
    WaypointWithCallback,
)
from res_plan_server.task_status import TaskStatus, TaskStatusUpdate
from res_plan_server.transport.transport_messages import (
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanErrorCode,
    PlanErrorMsg,
    PlanIdMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
)


class FakeTransport(ExecutorBaseTransport):
    def __init__(self) -> None:
        self.published_plan_errors: list[tuple[str, PlanErrorMsg]] = []
        self.published_task_statuses: list[TaskStatusUpdate] = []

    def subscribe_robot_onboarding(
        self, callback: Callable[[RobotOnboardMsg], None]
    ) -> None:
        pass

    def subscribe_participant_discovery(
        self, callback: Callable[[ParticipantDiscoveryMsg], None]
    ) -> None:
        pass

    def subscribe_plan(self, robot_id: str, callback: Callable[[Plan], None]) -> None:
        pass

    def subscribe_committed_locations_request(
        self, callback: Callable[[str], None]
    ) -> None:
        pass

    def publish_progress(self, robot_id: str, progress_msg: PlanProgressMsg) -> None:
        pass

    def publish_plan_error(self, robot_id: str, error_msg: PlanErrorMsg) -> None:
        self.published_plan_errors.append((robot_id, error_msg))

    def publish_committed_locations_response(
        self, response_msg: CommittedLocationsResponseMsg
    ) -> None:
        pass

    def publish_task_status(self, status_update: TaskStatusUpdate) -> None:
        self.published_task_statuses.append(status_update)


class FakeRobotController(BaseRobotController):
    def __init__(self) -> None:
        super().__init__(map_data=None)

    def enqueue(
        self, robot_id: str, waypoints_with_callbacks: list[WaypointWithCallback]
    ) -> None:
        pass

    def shutdown(self, interrupted: bool = False) -> None:
        pass


def _drain_one_message(executor: PlanExecutor) -> None:
    msg = executor._message_queue.get_nowait()
    assert msg[0] == "error"
    _, robot_id, error_code, reason = msg
    executor._handle_error(robot_id, error_code, reason)


def _make_plan(plan_id: PlanId) -> Plan:
    return Plan(
        waypoints=[
            Waypoint(
                name="A",
                position=[0, 0],
                progress=0.0,
                departure_blockers=[],
                departure_action="",
            ),
            Waypoint(
                name="B",
                position=[1, 0],
                progress=1.0,
                departure_blockers=[],
                departure_action="",
            ),
        ],
        start_time=None,
        plan_id=plan_id,
        workflow="",
    )


def test_handle_error_reports_failed_plans_id_not_a_cleared_one() -> None:
    """
    _handle_error must capture plan_id before calling on_plan_failed
    """
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = FakeRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    dm.set_plan("robot_0", _make_plan(plan_id))

    robot_controller._on_robot_failed(
        "robot_0", PlanErrorCode.PATH_BLOCKED, "obstacle detected on path"
    )
    _drain_one_message(executor)

    assert len(transport.published_plan_errors) == 1
    published_robot_id, error_msg = transport.published_plan_errors[0]
    assert published_robot_id == "robot_0"
    expected_plan_id_msg = PlanIdMsg(
        destination_session=str(plan_id.destination_session),
        plan_version=plan_id.plan_version,
    )
    assert error_msg.plan_id == expected_plan_id_msg, (
        "must report the plan that failed, not None"
    )
    assert error_msg.error_code == PlanErrorCode.PATH_BLOCKED
    assert error_msg.details == "obstacle detected on path"

    # on_plan_failed must still have run: the robot is removed from active tracking
    # so compute_commit_cut and dispatch no longer iterate it.
    assert dm.get_plan_state("robot_0") is None


def test_handle_error_publishes_failed_task_status() -> None:
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = FakeRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    dm.set_plan("robot_0", _make_plan(plan_id))

    robot_controller._on_robot_failed(
        "robot_0", PlanErrorCode.INCOMPATIBLE_ACTION, "unsupported action"
    )
    _drain_one_message(executor)

    assert len(transport.published_task_statuses) == 1
    status_update = transport.published_task_statuses[0]
    assert status_update.task_id == str(plan_id)
    assert status_update.robot_id == "robot_0"
    assert status_update.status == TaskStatus.FAILED
    assert status_update.reason == "unsupported action"


def test_handle_error_with_no_plan_state_does_not_publish_plan_error() -> None:
    """
    If the robot has no tracked plan at all, there's no plan_id to report;
    _handle_error must skip publish_plan_error rather than crash.
    """
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = FakeRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    robot_controller._on_robot_failed(
        "unknown_robot", PlanErrorCode.REPLAN_REQUEST, "no plan on file"
    )
    _drain_one_message(executor)

    assert transport.published_plan_errors == []
