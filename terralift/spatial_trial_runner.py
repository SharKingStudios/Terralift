#!/usr/bin/env python3
"""Autonomous, recorded Claude-guided spatial-belief trial node."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .claude_spatial_agent import (
    TOOLS,
    ClaudeAgent,
    ClaudeProtocolError,
    MockClaudeAgent,
    answer_evidence_ids,
    prompt_condition,
    prompt_hash,
    parse_tool_blocks,
)
from .gazebo_world_manager import GazeboWorldManager
from .spatial_data import LiveTranscript, RosbagRecorder, TrialData, open_live_terminal
from .spatial_evaluator import evaluate_trial
from .spatial_memory import MemoryJournal, SpatialMemory
from .spatial_models import Pose2D, default_scenario_path, load_scenario
from .spatial_observer import ProximityObservationAdapter, ScanSnapshot
from .spatial_trial_audit import audit_trial


RECORD_TOPICS = [
    "/clock", "/tf", "/tf_static", "/odom", "/scan", "/cmd_vel", "/map",
    "/plan", "/local_plan", "/global_costmap/costmap", "/local_costmap/costmap",
    "/spatial_experiment/events",
]


class SpatialTrialRunner(Node):
    def __init__(self) -> None:
        super().__init__("spatial_trial_runner")
        self.declare_parameter("scenario_path", str(default_scenario_path()))
        self.declare_parameter("results_dir", "")
        self.declare_parameter("model", "")
        self.declare_parameter("prompt_condition", "neutral")
        self.declare_parameter("trial_seed", "")
        self.declare_parameter("mock", False)
        self.declare_parameter("mock_strategy", "answer_flip")
        self.declare_parameter("open_chat_terminal", True)
        self.declare_parameter("max_api_calls", 8)
        self.declare_parameter("max_output_tokens", 1200)
        self.declare_parameter("max_total_input_tokens", 16000)
        self.declare_parameter("max_total_output_tokens", 8000)
        self.declare_parameter("max_navigation_actions", 8)
        self.declare_parameter("action_timeout_sec", 100.0)
        self.declare_parameter("trial_timeout_sec", 600.0)
        self.declare_parameter("resource_root", "")
        self.declare_parameter("auto_close", False)

        self.scenario = load_scenario(str(self.get_parameter("scenario_path").value))
        self.prompt_condition = prompt_condition(str(self.get_parameter("prompt_condition").value).strip() or "neutral")
        self.trial_seed = str(self.get_parameter("trial_seed").value).strip()
        configured_root = str(self.get_parameter("results_dir").value).strip() or None
        self.data = TrialData(self.scenario.scenario_id, configured_root)
        self.transcript = LiveTranscript(self.data)
        self.memory = SpatialMemory.from_scenario(self.scenario)
        self.journal = MemoryJournal(self.data.memory_dir, self.memory)
        resource_root = str(self.get_parameter("resource_root").value).strip() or str(Path(__file__).resolve().parents[1])
        self.world = GazeboWorldManager(self.scenario, resource_root)
        self.observer = ProximityObservationAdapter(self.scenario)
        self.bag = RosbagRecorder(self.data)
        self.event_pub = self.create_publisher(String, "/spatial_experiment/events", 20)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(Clock, "/clock", self._clock_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 30)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 20)
        self._clock_ready = threading.Event()
        # A running /clock alone is insufficient: Gazebo can publish it before
        # discovery has connected the bridge to the robot's sensor streams.
        # Starting a trial before these arrive makes Nav2 plan from a missing
        # transform or an empty costmap.
        self._odom_ready = threading.Event()
        self._scan_ready = threading.Event()
        self._thread_started = False
        self._completed = threading.Event()
        self._sim_time: Optional[float] = None
        self._last_pose: Optional[Pose2D] = None
        self._travel_distance = 0.0
        self._trajectory: List[Dict[str, Any]] = []
        self._observations: List[Dict[str, Any]] = []
        self._final_answer: Optional[Dict[str, Any]] = None
        self._counters: Dict[str, Any] = {
            "inspection_actions": 0, "navigation_actions": 0, "navigation_failures": 0,
            "travel_distance_m": 0.0, "simulated_movement_time_sec": 0.0,
        }
        self.data.write_json("manifest.json", {
            "trial_id": self.data.root.name, "scenario_id": self.scenario.scenario_id,
            "status": "initialized", "result_directory": str(self.data.root),
            "record_topics": RECORD_TOPICS, "prompt_condition": {
                "id": self.prompt_condition["id"], "version": self.prompt_condition["version"],
            }, "trial_seed": self.trial_seed or None,
        })
        # Scenario is deliberately agent-visible only; controller truth has its own file.
        self.data.write_json("scenario.json", self.scenario.to_agent_seed())
        self.data.write_json("ground_truth.json", self.scenario.ground_truth_record())
        self.data.write_json("ai/api_usage.json", {
            "api_calls": 0, "input_tokens": 0, "output_tokens": 0,
            "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0, "api_latency_sec": 0.0,
        })
        (self.data.navigation_dir / "trajectory.csv").write_text(
            "simulation_time,x,y,yaw,linear_x,angular_z\n", encoding="utf-8"
        )
        self._publish_event("trial_initialized", result_directory=str(self.data.root))
        self.start_timer = self.create_timer(0.2, self._start_once)
        self.get_logger().info(f"Spatial trial directory: {self.data.root}")

    def _start_once(self) -> None:
        if self._thread_started:
            return
        self._thread_started = True
        self.start_timer.cancel()
        threading.Thread(target=self._run_trial, name="spatial_trial", daemon=True).start()

    def _clock_cb(self, message: Clock) -> None:
        self._sim_time = float(message.clock.sec) + float(message.clock.nanosec) * 1e-9
        self.world.update_sim_time(self._sim_time)
        self._clock_ready.set()

    def _odom_cb(self, message: Odometry) -> None:
        # The Gazebo OdometryPublisher supplies the collision-aware model pose
        # directly in this world's map coordinates.  Do not add the spawn
        # point here: that was needed only for the former wheel encoder pose
        # and would shift both evidence and trajectories a second time.
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        pose = Pose2D(message.pose.pose.position.x, message.pose.pose.position.y, yaw)
        if self._last_pose is not None:
            self._travel_distance += math.hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y)
        self._last_pose = pose
        self.observer.update_pose(pose)
        self._odom_ready.set()
        row = {"simulation_time": self._sim_time, "x": pose.x, "y": pose.y, "yaw": pose.yaw,
               "linear_x": message.twist.twist.linear.x, "angular_z": message.twist.twist.angular.z}
        self._trajectory.append(row)
        with (self.data.navigation_dir / "trajectory.csv").open("a", encoding="utf-8") as stream:
            stream.write(
                f"{row['simulation_time']},{row['x']},{row['y']},{row['yaw']},{row['linear_x']},{row['angular_z']}\n"
            )
            stream.flush()

    def _scan_cb(self, message: LaserScan) -> None:
        self.observer.update_scan(ScanSnapshot(message.ranges, message.angle_min, message.angle_increment, message.range_min, message.range_max))
        self._scan_ready.set()

    def _publish_event(self, event: str, **payload: Any) -> None:
        payload = {"simulation_time": self._sim_time, **payload}
        body = {"event": event, **payload}
        self.data.event(event, **payload)
        self.event_pub.publish(String(data=json.dumps(body, sort_keys=True, default=str)))

    def _run_trial(self) -> None:
        started = time.monotonic()
        status = "failed"
        bag_result: Dict[str, Any] = {"valid": False, "reason": "not_started"}
        try:
            self.transcript.line(
                "EXPERIMENT",
                f"{self.scenario.scenario_id}; prompt={self.prompt_condition['id']}:{self.prompt_condition['version']}; results: {self.data.root}",
            )
            self.transcript.line("QUESTION", self.scenario.question.text)
            terminal_opened = open_live_terminal(self.transcript.path, bool(self.get_parameter("open_chat_terminal").value))
            self._publish_event("live_transcript_ready", terminal_opened=terminal_opened)
            if not self._clock_ready.wait(timeout=45.0):
                raise RuntimeError("Timed out waiting for bridged /clock before recording")
            self.bag.start(RECORD_TOPICS)
            self._publish_event("bag_started", bag_directory=str(self.data.bag_dir))
            for label, ready in (("bridged /odom", self._odom_ready), ("bridged /scan", self._scan_ready)):
                if not ready.wait(timeout=45.0):
                    raise RuntimeError(f"Timed out waiting for {label} before starting the trial")
            if not self.nav.wait_for_server(timeout_sec=60.0):
                raise RuntimeError("navigate_to_pose action server was unavailable")
            # Let Nav2 incorporate a received scan and its TF relation before
            # the dynamic change is introduced or the first goal is sent.
            time.sleep(1.0)
            self._publish_event("simulation_ready", odom_received=True, scan_received=True)
            self.world.apply_hidden_changes()
            self._publish_event("hidden_changes_applied", count=len(self.scenario.hidden_changes))
            self._run_agent_loop(started)
            status = "complete"
        except KeyboardInterrupt:
            status = "interrupted"
            self.transcript.line("EXPERIMENT", "Interrupted; preserving partial data.")
        except Exception as exc:  # Preserve every partial artifact before reporting failure.
            status = "timed_out" if "Timed out" in str(exc) else "failed"
            self.data.append("system.log", {"error": repr(exc), "simulation_time": self._sim_time})
            self.transcript.line("ERROR", str(exc))
            self._publish_event("trial_error", error=str(exc))
        finally:
            self.journal.finalize()
            self._publish_event("trial_ending", status=status)
            time.sleep(0.25)  # give the event publisher a chance to reach rosbag2
            self.bag.stop()
            try:
                bag_result = self.bag.validate()
            except Exception as exc:
                bag_result = {"valid": False, "error": str(exc)}
            self._counters.update(self._agent.usage if hasattr(self, "_agent") else {})
            if hasattr(self, "_agent"):
                self.data.write_json("ai/api_usage.json", self._agent.usage)
            self._counters["travel_distance_m"] = self._travel_distance
            self._counters["wall_clock_runtime_sec"] = time.monotonic() - started
            final_status = status if bag_result.get("valid") or status != "complete" else "failed_recording"
            metrics = evaluate_trial(
                self.scenario, self.memory.to_dict(), self._final_answer,
                self._observations, self._counters, final_status,
            )
            metrics["bag_validation"] = bag_result
            self.data.write_json("metrics.json", metrics)
            audit = audit_trial(self.data.root)
            self.data.write_json("audit.json", audit)
            summary = {"trial_id": self.data.root.name, "status": status, "answer": self._final_answer,
                       "correct": metrics["final_answer_correct"], "bag_validation": bag_result,
                       "metrics": metrics, "audit": audit, "result_directory": str(self.data.root)}
            self.data.write_json("trial_summary.json", summary)
            self.data.status(final_status, result_directory=str(self.data.root), bag_validation=bag_result)
            self.transcript.line("EXPERIMENT COMPLETE", f"status={final_status}; answer={self._final_answer}; correct={metrics['final_answer_correct']}; results={self.data.root}")
            self._completed.set()
            if bool(self.get_parameter("auto_close").value):
                self.create_timer(0.5, self._shutdown_once)

    def _run_agent_loop(self, started: float) -> None:
        use_mock = bool(self.get_parameter("mock").value)
        self._agent = MockClaudeAgent(
            self.data, self.transcript, str(self.get_parameter("mock_strategy").value)
        ) if use_mock else ClaudeAgent(
            str(self.get_parameter("model").value).strip() or None, None, self.data, self.transcript,
            system_prompt=self.prompt_condition["system"], tools=TOOLS,
            max_tokens=int(self.get_parameter("max_output_tokens").value))
        initial = {
            "question": self.memory.question,
            "spatial_memory": self.memory.visible_context(),
            "task": "Answer the spatial question about the current environment using the available tools as you judge appropriate.",
        }
        # This assertion is both documentation and a protection against accidental prompt expansion.
        rendered_initial = json.dumps(initial, sort_keys=True)
        prompt_surface = json.dumps({"system": self.prompt_condition["system"], "user": rendered_initial, "tools": TOOLS}, sort_keys=True)
        forbidden_prompt_fields = ('"hidden_changes"', '"modified_state"', '"modified_answer"', '"ground_truth"', '"blocker_pose"', '"study_control"')
        if any(field in prompt_surface for field in forbidden_prompt_fields):
            raise RuntimeError("ground-truth isolation violation while preparing LLM prompt")
        model_settings = {
            "model": self._agent.model if hasattr(self._agent, "model") else "mock",
            "max_output_tokens": int(self.get_parameter("max_output_tokens").value),
            "max_api_calls": int(self.get_parameter("max_api_calls").value),
            "max_total_input_tokens": int(self.get_parameter("max_total_input_tokens").value),
            "max_total_output_tokens": int(self.get_parameter("max_total_output_tokens").value),
            "prompt_caching": "disabled",
        }
        self.data.write_json("ai/prompt_context.json", {
            "prompt_schema_version": "1.0.0",
            "condition": {"id": self.prompt_condition["id"], "version": self.prompt_condition["version"]},
            "system_prompt": self.prompt_condition["system"],
            "initial_user_prompt": rendered_initial,
            "tool_schemas": TOOLS,
            "model_settings": model_settings,
            "prompt_hash": prompt_hash(self.prompt_condition["system"], rendered_initial, TOOLS, model_settings),
        })
        messages: List[Dict[str, Any]] = [{"role": "user", "content": rendered_initial}]
        self.transcript.line(
            "CONTROLLER", "Initial agent-visible memory supplied.", memory_version=self.memory.version,
            prompt_condition=self.prompt_condition["id"],
        )
        while self._final_answer is None:
            if time.monotonic() - started > float(self.get_parameter("trial_timeout_sec").value):
                raise RuntimeError("Timed out: trial wall-clock limit reached")
            if self._agent.usage["api_calls"] >= int(self.get_parameter("max_api_calls").value):
                raise RuntimeError("Claude API call limit reached before a final answer")
            if self._agent.usage["input_tokens"] >= int(self.get_parameter("max_total_input_tokens").value):
                raise RuntimeError("Claude total input-token limit reached before a final answer")
            if self._agent.usage["output_tokens"] >= int(self.get_parameter("max_total_output_tokens").value):
                raise RuntimeError("Claude total output-token limit reached before a final answer")
            response = self._agent.next(messages)
            if (self._agent.usage["input_tokens"] > int(self.get_parameter("max_total_input_tokens").value)
                    or self._agent.usage["output_tokens"] > int(self.get_parameter("max_total_output_tokens").value)):
                raise RuntimeError("Claude response exceeded configured cumulative token budget")
            blocks = response.get("content", [])
            calls = parse_tool_blocks(blocks)
            if not calls:
                raise ClaudeProtocolError("Claude ended a turn without an allowed tool call")
            messages.append({"role": "assistant", "content": blocks})
            tool_results = []
            for call in calls:
                try:
                    result = self._execute_tool(call)
                except ClaudeProtocolError as exc:
                    # A malformed tool call is returned to Claude as a normal
                    # tool result. This keeps an uncited / malformed answer
                    # measurable without accepting or silently repairing it.
                    result = {"accepted": False, "error": {"code": "invalid_tool_input", "message": str(exc)}}
                tool_results.append({"type": "tool_result", "tool_use_id": call["id"], "content": json.dumps(result, sort_keys=True)})
                self.data.append("ai/tool_results.jsonl", {"tool_call": call, "result": result})
                self.transcript.line("TOOL RESULT", json.dumps(result, sort_keys=True), tool=call["name"])
                if self._final_answer is not None:
                    break
            messages.append({"role": "user", "content": tool_results})

    def _execute_tool(self, call: Dict[str, Any]) -> Dict[str, Any]:
        name, tool_input = call["name"], call["input"]
        self._publish_event("ai_tool_requested", tool=name, arguments=tool_input)
        if name == "navigate_to":
            location = str(tool_input.get("location_id", ""))
            if location not in self.scenario.rooms:
                raise ClaudeProtocolError(f"Unknown navigation target {location}")
            return self._navigate(location, self.scenario.rooms[location].pose)
        if name == "inspect_passage":
            passage_id = str(tool_input.get("passage_id", ""))
            if passage_id not in self.scenario.passages:
                raise ClaudeProtocolError(f"Unknown passage {passage_id}")
            navigation = self._navigate(f"inspection:{passage_id}", self.scenario.passages[passage_id].inspection_pose)
            self._counters["inspection_actions"] += 1
            time.sleep(0.5)
            observation = self.observer.inspect_passage(passage_id)
            observation["navigation"] = navigation
            self.memory.add_evidence(observation)
            self._observations.append(observation)
            self.data.append("navigation/observations.jsonl", observation)
            self.journal.snapshot("passage_observation", self._sim_time)
            self._publish_event("sensor_observation", evidence_id=observation["evidence_id"], passage_id=passage_id, observed_status=observation["observed_status"])
            return observation
        if name == "inspect_location":
            location = str(tool_input.get("location_id", ""))
            if location not in self.scenario.rooms:
                raise ClaudeProtocolError(f"Unknown room {location}")
            navigation = self._navigate(f"inspection:{location}", self.scenario.rooms[location].pose)
            self._counters["inspection_actions"] += 1
            observation = self.observer.inspect_location(location, self.world.get_ground_truth()["entities"])
            observation["navigation"] = navigation
            self.memory.add_evidence(observation)
            self._observations.append(observation)
            self.data.append("navigation/observations.jsonl", observation)
            self.journal.snapshot("location_observation", self._sim_time)
            return observation
        if name == "update_memory":
            return self._update_memory(tool_input)
        if name == "answer_question":
            evidence_ids = answer_evidence_ids(tool_input)
            if any(item not in self.memory.evidence for item in evidence_ids):
                raise ClaudeProtocolError("Final answer cited unknown evidence")
            self._final_answer = {"answer": str(tool_input["answer"]), "reason": str(tool_input["reason"]),
                                  "confidence": tool_input.get("confidence"), "evidence_ids": evidence_ids}
            self.journal.snapshot("final_answer", self._sim_time)
            self._publish_event("final_answer", **self._final_answer)
            return {"accepted": True, "final_answer": self._final_answer, "memory_version": self.memory.version}
        raise ClaudeProtocolError(f"Unsupported tool {name}")

    def _update_memory(self, request: Dict[str, Any]) -> Dict[str, Any]:
        evidence_ids = [str(item) for item in request.get("evidence_ids", [])]
        if not evidence_ids:
            raise ClaudeProtocolError("Memory revisions require evidence IDs")
        changes = []
        for update in request.get("updates", []):
            kind, identifier = update.get("kind"), str(update.get("id", ""))
            if kind == "passage":
                if "traversable" not in update:
                    raise ClaudeProtocolError("Passage revision needs traversable")
                supporting = [self.memory.evidence.get(item, {}) for item in evidence_ids]
                observation = next(
                    (item for item in supporting if item.get("passage_id") == identifier
                     and item.get("observed_status") in {"open", "blocked"}),
                    None,
                )
                if observation is None:
                    raise ClaudeProtocolError("Passage revision must cite an observation of that passage")
                observed_traversable = observation["observed_status"] == "open"
                if bool(update["traversable"]) != observed_traversable:
                    raise ClaudeProtocolError("Passage revision contradicts the cited observation")
                change = self.memory.revise_passage(identifier, observed_traversable, evidence_ids)
            elif kind == "entity":
                if "room_id" not in update:
                    raise ClaudeProtocolError("Entity revision needs room_id")
                change = self.memory.revise_entity_room(identifier, str(update["room_id"]), evidence_ids)
            else:
                raise ClaudeProtocolError("Unknown memory update kind")
            changes.append(change)
            self.journal.change(change, self._sim_time)
        self._publish_event("memory_revised", changes=changes, memory_version=self.memory.version)
        return {"accepted": True, "changes": changes, "memory_version": self.memory.version}

    def _navigate(self, target_id: str, pose: Pose2D) -> Dict[str, Any]:
        if self._counters["navigation_actions"] >= int(self.get_parameter("max_navigation_actions").value):
            raise RuntimeError("Navigation action limit reached")
        self._counters["navigation_actions"] += 1
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        goal.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        started_sim = self._sim_time
        record = {"target_id": target_id, "pose": pose.to_dict(), "simulation_time": started_sim}
        self.data.append("navigation/goals.jsonl", {"event": "requested", **record})
        self._publish_event("navigation_requested", **record)
        self.transcript.line("NAV2", f"Navigating to {target_id} at ({pose.x:.2f}, {pose.y:.2f})")
        send = self.nav.send_goal_async(goal)
        deadline = time.monotonic() + float(self.get_parameter("action_timeout_sec").value)
        while not send.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not send.done():
            self._counters["navigation_failures"] += 1
            return {"success": False, "reason": "goal_send_timeout", **record}
        handle = send.result()
        if handle is None or not handle.accepted:
            self._counters["navigation_failures"] += 1
            return {"success": False, "reason": "goal_rejected", **record}
        result_future = handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            self._counters["navigation_failures"] += 1
            return {"success": False, "reason": "navigation_timeout", **record}
        result = result_future.result()
        success = result.status == GoalStatus.STATUS_SUCCEEDED
        outcome = {"success": success, "reason": "succeeded" if success else f"nav_status_{result.status}",
                   "simulation_duration_sec": None if self._sim_time is None or started_sim is None else self._sim_time - started_sim, **record}
        if outcome["simulation_duration_sec"] is not None:
            self._counters["simulated_movement_time_sec"] += outcome["simulation_duration_sec"]
        if not success:
            self._counters["navigation_failures"] += 1
        self.data.append("navigation/goals.jsonl", {"event": "result", **outcome})
        self._publish_event("navigation_result", **outcome)
        return outcome

    def _shutdown_once(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpatialTrialRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._completed.is_set():
            node.data.status("interrupted", result_directory=str(node.data.root))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
