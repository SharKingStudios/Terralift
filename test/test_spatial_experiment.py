"""Focused tests that do not require ROS, Gazebo, or Anthropic credentials."""

from __future__ import annotations

import json
import runpy
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET

from terralift.claude_spatial_agent import ClaudeProtocolError, PROMPT_CONDITIONS, answer_evidence_ids, parse_tool_blocks, prompt_condition, prompt_hash
from terralift.spatial_data import TrialData
from terralift.spatial_evaluator import evaluate_trial
from terralift.spatial_memory import MemoryJournal, SpatialMemory
from terralift.spatial_models import default_scenario_path, load_scenario
from terralift.spatial_observer import ProximityObservationAdapter, ScanSnapshot


def scenario():
    return load_scenario(default_scenario_path())


def test_distance_ranking_flip_and_question_definition():
    subject = scenario()
    assert subject.question.family == "semantic_distance_ranking"
    assert subject.answer_for_state(subject.initial_state())["answer"] == "D"
    modified = subject.answer_for_state(subject.modified_state())
    assert modified["answer"] == "E"
    assert modified["distances"]["D"] == 3.0
    assert modified["distances"]["E"] == 4.0


def test_control_scenarios_have_declared_and_independently_validated_effects():
    root = Path(__file__).parents[1] / "config" / "spatial_scenarios"
    expectations = {
        "distance_ranking_flip.json": ("topological", "flip", "D", "E"),
        "topology_answer_invariant_bc_block.json": ("topological", "invariant", "D", "D"),
        "no_change_control.json": ("none", "invariant", "D", "D"),
        "semantic_answer_flip_move_b_d_to_c.json": ("semantic", "flip", "D", "C"),
        "semantic_answer_invariant_move_b_e_to_b.json": ("semantic", "invariant", "D", "D"),
    }
    for filename, (kind, effect, before, after) in expectations.items():
        subject = load_scenario(root / filename)
        assert subject.study_control == {"change_kind": kind, "answer_effect": effect}
        assert subject.answer_for_state(subject.initial_state())["answer"] == before
        assert subject.answer_for_state(subject.modified_state())["answer"] == after


def test_neutral_prompt_has_no_verification_intervention_and_auditable_hash():
    neutral = prompt_condition("neutral")
    assert neutral["id"] == "neutral"
    assert "outdated" not in neutral["system"].lower()
    assert "contradict" not in neutral["system"].lower()
    assert "verify" not in neutral["system"].lower()
    guided = prompt_condition("verification_guided")
    assert "verification" in guided["system"].lower()
    rendered = '{"question":"example"}'
    settings = {"model": "claude-haiku-5-5", "max_output_tokens": 1200}
    assert prompt_hash(neutral["system"], rendered, [], settings) == prompt_hash(neutral["system"], rendered, [], settings)
    assert set(PROMPT_CONDITIONS) == {"neutral", "verification_guided"}


def test_agent_seed_has_no_hidden_ground_truth():
    seed = scenario().to_agent_seed()
    encoded = json.dumps(seed)
    assert "hidden_changes" not in encoded
    assert "blocker_pose" not in encoded
    assert "modified_state" not in encoded
    assert "scenario_id" not in seed
    assert seed["passages"]["A_E"]["traversable"] is True


def test_visible_memory_excludes_descriptive_scenario_identity():
    memory = SpatialMemory.from_scenario(scenario())
    visible = memory.visible_context()
    assert "scenario_id" not in visible
    assert "distance_ranking_flip" not in json.dumps(visible)


def test_memory_journal_persists_complete_revision_history(tmp_path: Path):
    memory = SpatialMemory.from_scenario(scenario())
    journal = MemoryJournal(tmp_path, memory)
    memory.add_evidence({"evidence_id": "observation_001", "passage_id": "A_E", "observed_status": "blocked"})
    change = memory.revise_passage("A_E", False, ["observation_001"])
    journal.change(change, 12.5)
    journal.finalize()
    assert json.loads((tmp_path / "initial_memory.json").read_text())["passages"]["A_E"]["traversable"] is True
    assert json.loads((tmp_path / "final_memory.json").read_text())["passages"]["A_E"]["traversable"] is False
    assert len((tmp_path / "snapshots.jsonl").read_text().splitlines()) >= 2
    assert "A_E" in (tmp_path / "changes.jsonl").read_text()


def test_tool_call_parser_rejects_unknown_tools():
    parsed = parse_tool_blocks([{"type": "tool_use", "id": "one", "name": "inspect_passage", "input": {"passage_id": "A_E"}}])
    assert parsed[0]["name"] == "inspect_passage"
    try:
        parse_tool_blocks([{"type": "tool_use", "id": "two", "name": "leak_truth", "input": {}}])
    except ClaudeProtocolError:
        pass
    else:
        raise AssertionError("Unknown tool was accepted")


def test_final_answer_requires_a_structured_evidence_array_but_allows_empty_citations():
    assert answer_evidence_ids({"evidence_ids": []}) == []
    assert answer_evidence_ids({"evidence_ids": ["observation_001"]}) == ["observation_001"]
    for malformed in ({}, {"evidence_ids": "observation_001"}, {"evidence_ids": [1]}):
        try:
            answer_evidence_ids(malformed)
        except ClaudeProtocolError:
            pass
        else:
            raise AssertionError(f"Malformed final-answer citations were accepted: {malformed!r}")


def test_trial_directory_isolated_and_partial_data_survives(tmp_path: Path):
    first = TrialData("distance_ranking_flip", str(tmp_path))
    first.event("trial_initialized")
    first.status("failed", reason="simulated_failure")
    second = TrialData("distance_ranking_flip", str(tmp_path))
    assert first.root != second.root
    assert (first.root / "events.jsonl").is_file()
    assert json.loads((first.root / "status.json").read_text())["status"] == "failed"


def test_evaluator_separates_belief_revision_from_answer_correctness():
    subject = scenario()
    memory = SpatialMemory.from_scenario(subject)
    evidence = {"evidence_id": "observation_001", "passage_id": "A_E", "observed_status": "blocked"}
    memory.add_evidence(evidence)
    memory.revise_passage("A_E", False, ["observation_001"])
    metrics = evaluate_trial(
        subject, memory.to_dict(), {"answer": "E", "evidence_ids": ["observation_001"]}, [evidence],
        {"inspection_actions": 1, "navigation_actions": 1}, "complete",
    )
    assert metrics["answer_changed"] is True
    assert metrics["final_answer_correct"] is True
    assert metrics["changed_edge_revised_correctly"] is True
    assert metrics["answer_consistent_with_final_memory"] is True
    assert metrics["memory_accuracy"] == 1.0
    assert metrics["verification_coverage"] == 1.0


def test_correct_answer_without_evidence_is_not_scored_as_evidence_supported():
    subject = scenario()
    memory = SpatialMemory.from_scenario(subject)
    metrics = evaluate_trial(subject, memory.to_dict(), {"answer": "E", "evidence_ids": []}, [], {}, "complete")
    assert metrics["final_answer_correct"] is True
    assert metrics["answer_supported_by_observed_evidence"] is False
    assert metrics["verification_coverage"] == 0.0


def test_unchanged_control_does_not_treat_a_cited_selective_check_as_sufficient_evidence():
    subject = load_scenario(Path(__file__).parents[1] / "config" / "spatial_scenarios" / "no_change_control.json")
    memory = SpatialMemory.from_scenario(subject)
    evidence = {"evidence_id": "observation_001", "passage_id": "A_E", "observed_status": "open"}
    metrics = evaluate_trial(subject, memory.to_dict(), {"answer": "D", "evidence_ids": ["observation_001"]}, [evidence], {}, "complete")
    assert metrics["final_answer_correct"] is True
    assert metrics["cited_evidence_is_observation_accurate"] is True
    assert metrics["answer_supported_by_observed_evidence"] is False


def test_passage_inspection_uses_nearest_valid_lidar_return_at_the_pose():
    subject = scenario()
    adapter = ProximityObservationAdapter(subject)
    adapter.update_pose(subject.passages["A_E"].inspection_pose)
    adapter.update_scan(ScanSnapshot(
        ranges=[7.0, 0.8, float("inf")], angle_min=-3.14,
        angle_increment=3.14, range_min=0.12, range_max=12.0,
    ))
    observed = adapter.inspect_passage("A_E")
    assert observed["observed_status"] == "blocked"
    assert observed["source"] == "nearby_lidar"
    assert observed["nearest_lidar_range_m"] == 0.8


def test_passage_inspection_ignores_nearby_corridor_side_walls():
    subject = scenario()
    adapter = ProximityObservationAdapter(subject)
    adapter.update_pose(subject.passages["A_E"].inspection_pose)
    # The 0.5 m returns are side walls at +/- 90 degrees. The forward passage
    # direction is clear at 5 m, so this must remain an open observation.
    adapter.update_scan(ScanSnapshot(
        ranges=[0.5, 5.0, 0.5], angle_min=-1.5708,
        angle_increment=1.5708, range_min=0.12, range_max=12.0,
    ))
    observed = adapter.inspect_passage("A_E")
    assert observed["observed_status"] == "open"
    assert observed["nearest_lidar_range_m"] == 5.0


def test_passage_inspection_refuses_an_off_heading_lidar_reading():
    subject = scenario()
    adapter = ProximityObservationAdapter(subject)
    pose = subject.passages["C_D"].inspection_pose
    adapter.update_pose(type(pose)(pose.x, pose.y, pose.yaw + 0.25))
    adapter.update_scan(ScanSnapshot(
        ranges=[0.2, 5.0, 0.2], angle_min=-1.5708,
        angle_increment=1.5708, range_min=0.12, range_max=12.0,
    ))
    observed = adapter.inspect_passage("C_D")
    assert observed["observed_status"] == "unobserved"
    assert observed["reason"] == "inspection_heading_outside_tolerance"


def test_passage_inspection_treats_infinite_local_lidar_rays_as_clear_to_range():
    subject = scenario()
    adapter = ProximityObservationAdapter(subject)
    adapter.update_pose(subject.passages["B_C"].inspection_pose)
    adapter.update_scan(ScanSnapshot(
        ranges=[float("inf"), float("inf"), float("inf")], angle_min=-0.2,
        angle_increment=0.2, range_min=0.12, range_max=12.0,
    ))
    observed = adapter.inspect_passage("B_C")
    assert observed["observed_status"] == "open"
    assert observed["nearest_lidar_range_m"] == 12.0


def test_robot_wheel_axes_match_the_rolled_wheel_link_frame():
    root = ET.parse(Path(__file__).parents[1] / "models" / "terralift_mecanum" / "model.sdf").getroot()
    axes = [joint.find("axis/xyz") for joint in root.findall(".//joint") if joint.attrib["name"].endswith("_joint")]
    assert len(axes) == 4
    assert all(axis is not None and axis.text.strip() == "0 0 -1" for axis in axes)


def test_lidar_stays_behind_the_chassis_collision_front_at_barrier_contact():
    root = ET.parse(Path(__file__).parents[1] / "models" / "terralift_mecanum" / "model.sdf").getroot()
    chassis_size = list(map(float, root.findtext("./model/link[@name='base_link']/collision/geometry/box/size").split()))
    scanner_pose = list(map(float, root.findtext("./model/link[@name='base_link']/sensor[@name='scan_sensor']/pose").split()))
    # The collision front is x=+0.45 m. A scanner at +0.30 m cannot be
    # embedded in a wall at the first chassis contact.
    assert scanner_pose[0] == 0.30
    assert scanner_pose[0] < chassis_size[0] / 2
    launch_source = (Path(__file__).parents[1] / "launch" / "spatial_experiment.launch.py").read_text(encoding="utf-8")
    observer_source = (Path(__file__).parents[1] / "terralift" / "spatial_observer.py").read_text(encoding="utf-8")
    assert 'arguments=["0.30", "0", "0.60"' in launch_source
    assert "scanner_forward_offset_m: float = 0.30" in observer_source


def test_navigation_odom_uses_collision_aware_model_pose_not_wheel_encoder():
    """A stalled physical robot must never advance in RViz/Nav2 odometry."""
    root = ET.parse(Path(__file__).parents[1] / "models" / "terralift_mecanum" / "model.sdf").getroot()
    diff_drive = root.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    model_pose_odom = root.find(".//plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert diff_drive is not None
    assert diff_drive.findtext("odom_topic") == "/wheel_odom"
    assert model_pose_odom is not None
    assert model_pose_odom.findtext("odom_topic") == "/odom"
    assert model_pose_odom.findtext("odom_frame") == "odom"
    assert model_pose_odom.findtext("robot_base_frame") == "base_link"
    assert model_pose_odom.findtext("dimensions") == "2"


def test_collision_aware_odom_uses_the_map_origin_without_a_second_spawn_offset():
    root = Path(__file__).parents[1]
    launch_source = (root / "launch" / "spatial_experiment.launch.py").read_text(encoding="utf-8")
    runner_source = (root / "terralift" / "spatial_trial_runner.py").read_text(encoding="utf-8")
    assert 'map_to_odom_x", default_value="0.0"' in launch_source
    assert 'map_to_odom_y", default_value="0.0"' in launch_source
    assert "Pose2D(message.pose.pose.position.x, message.pose.pose.position.y, yaw)" in runner_source
    assert "message.pose.pose.position.x + self.scenario.robot_start.x" not in runner_source


def test_physical_tool_calls_are_serialized_for_observation_conditioned_replanning():
    runner_source = (Path(__file__).parents[1] / "terralift" / "spatial_trial_runner.py").read_text(encoding="utf-8")
    assert 'PHYSICAL_ACTION_TOOLS = {"navigate_to", "inspect_passage", "inspect_location"}' in runner_source
    assert '"code": "physical_action_deferred"' in runner_source


def test_static_map_matches_resized_physical_walls_and_scenario(tmp_path: Path):
    """Catch a resized map/scenario drifting out of sync with Gazebo geometry."""
    root = Path(__file__).parents[1]
    generator = runpy.run_path(str(root / "scripts" / "generate_spatial_map.py"))
    regions = generator["FREE_REGIONS"]
    pgm_path = tmp_path / "spatial_map.pgm"
    generator["write_map"](pgm_path, pgm_path.with_suffix(".yaml"))
    pgm_tokens = [token for token in pgm_path.read_text(encoding="ascii").split() if token != "#"]
    # The generated P2 header is fixed and its only comment is `# generated
    # by Terralift`, whose individual words are harmlessly skipped below.
    assert pgm_tokens[:4] == ["P2", "generated", "by", "Terralift"]
    width, height, maximum = map(int, pgm_tokens[4:7])
    assert maximum == 255
    pixels = list(map(int, pgm_tokens[7:]))
    assert len(pixels) == width * height
    world = ET.parse(root / "worlds" / "spatial_warehouse.sdf").getroot()
    walls = []
    for model in world.findall("./world/model"):
        if model.attrib["name"] == "ground":
            continue
        for link in model.findall("./link"):
            for collision in link.findall("collision"):
                sx, sy, _ = map(float, collision.findtext("geometry/box/size").split())
                pose_text = collision.findtext("pose") or link.findtext("pose")
                x, y, *_ = map(float, pose_text.split())
                walls.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))

    def inside(x, y, rectangle):
        left, right, bottom, top = rectangle
        return left < x < right and bottom < y < top

    # Sample cell interiors across the building, avoiding exact wall boundaries.
    for ix in range(168):
        for iy in range(128):
            x, y = -21 + (ix + 0.5) * 0.25, -16 + (iy + 0.5) * 0.25
            region_free = any(inside(x, y, region) for region in regions)
            pixel_x = int((x - generator["ORIGIN_X"]) / generator["RESOLUTION"])
            pixel_y = int((y - generator["ORIGIN_Y"]) / generator["RESOLUTION"])
            mapped_free = pixels[(height - 1 - pixel_y) * width + pixel_x] == 254
            physical_wall = any(inside(x, y, wall) for wall in walls)
            # The static map is the navigation authority. Every map-free cell
            # must be physically clear; walls may cover additional exterior
            # space outside the deliberately enclosed floor plan.
            assert not (mapped_free and physical_wall), (x, y)
            assert not (mapped_free and not region_free), (x, y)

    subject = scenario()
    spawn = list(map(float, world.findtext("./world/include/pose").split()))
    assert spawn[:2] == [subject.robot_start.x, subject.robot_start.y]
    for room in subject.rooms.values():
        pixel_x = int((room.pose.x - generator["ORIGIN_X"]) / generator["RESOLUTION"])
        pixel_y = int((room.pose.y - generator["ORIGIN_Y"]) / generator["RESOLUTION"])
        assert pixels[(height - 1 - pixel_y) * width + pixel_x] == 254, room.room_id
        assert not any(inside(room.pose.x, room.pose.y, wall) for wall in walls), room.room_id
    for passage in subject.passages.values():
        pose, barrier = passage.inspection_pose, passage.blocker_pose
        assert any(inside(pose.x, pose.y, region) for region in regions)
        assert abs((pose.x - barrier.x) ** 2 + (pose.y - barrier.y) ** 2 - 4.0) < 1e-6
    blocker = ET.parse(root / "models" / "passage_blocker" / "model.sdf")
    barrier_width = float(blocker.findtext(".//collision/geometry/box/size").split()[1])
    for left, right, bottom, top in regions[5:]:
        # It must be wider than the physical robot's padded 1.02 m footprint,
        # yet fit entirely inside each 2.4 m corridor rather than intersecting
        # the permanent side walls.
        assert 1.02 < barrier_width < min(right - left, top - bottom)


def test_each_declared_passage_has_a_direct_wall_free_static_map_route(tmp_path: Path):
    """Prevent an L-corridor side wall from accidentally sealing its turn.

    Two rooms can remain globally connected by the rest of the cycle even if
    their declared direct passage is sealed. Search only the two endpoint
    rooms and that passage's own floor rectangles, so this catches exactly
    that error.
    """
    root = Path(__file__).parents[1]
    generator = runpy.run_path(str(root / "scripts" / "generate_spatial_map.py"))
    regions = generator["FREE_REGIONS"]
    pgm_path = tmp_path / "connectivity_map.pgm"
    generator["write_map"](pgm_path, pgm_path.with_suffix(".yaml"))
    tokens = [token for token in pgm_path.read_text(encoding="ascii").split() if token != "#"]
    width, height = map(int, tokens[4:6])
    pixels = list(map(int, tokens[7:]))
    subject = scenario()
    region_indexes = {
        "A_B": (0, 1, 5), "B_C": (1, 2, 6), "C_D": (2, 3, 7, 8),
        "D_E": (3, 4, 9, 10), "A_E": (0, 4, 11),
    }

    def cell_for(x: float, y: float) -> tuple[int, int]:
        return (
            int((x - generator["ORIGIN_X"]) / generator["RESOLUTION"]),
            int((y - generator["ORIGIN_Y"]) / generator["RESOLUTION"]),
        )

    def free_and_allowed(cell: tuple[int, int], allowed_regions) -> bool:
        px, py = cell
        if not (0 <= px < width and 0 <= py < height):
            return False
        if pixels[(height - 1 - py) * width + px] != 254:
            return False
        x = generator["ORIGIN_X"] + (px + 0.5) * generator["RESOLUTION"]
        y = generator["ORIGIN_Y"] + (py + 0.5) * generator["RESOLUTION"]
        return any(left < x < right and bottom < y < top for left, right, bottom, top in allowed_regions)

    for passage_id, indexes in region_indexes.items():
        passage = subject.passages[passage_id]
        start_room, end_room = (subject.rooms[room_id] for room_id in passage.rooms)
        start, target = cell_for(start_room.pose.x, start_room.pose.y), cell_for(end_room.pose.x, end_room.pose.y)
        allowed_regions = [regions[index] for index in indexes]
        assert free_and_allowed(start, allowed_regions), passage_id
        assert free_and_allowed(target, allowed_regions), passage_id
        queue, visited = deque([start]), {start}
        while queue and target not in visited:
            px, py = queue.popleft()
            for candidate in ((px - 1, py), (px + 1, py), (px, py - 1), (px, py + 1)):
                if candidate not in visited and free_and_allowed(candidate, allowed_regions):
                    visited.add(candidate)
                    queue.append(candidate)
        assert target in visited, f"{passage_id} is sealed by permanent wall geometry"
