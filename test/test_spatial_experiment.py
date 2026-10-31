"""Focused tests that do not require ROS, Gazebo, or Anthropic credentials."""

from __future__ import annotations

import json
import runpy
from pathlib import Path
import xml.etree.ElementTree as ET

from terralift.claude_spatial_agent import ClaudeProtocolError, parse_tool_blocks
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
    assert modified["distances"] == {"D": 3.0, "E": 4.0}


def test_agent_seed_has_no_hidden_ground_truth():
    seed = scenario().to_agent_seed()
    encoded = json.dumps(seed)
    assert "hidden_changes" not in encoded
    assert "blocker_pose" not in encoded
    assert "modified_state" not in encoded
    assert seed["passages"]["A_E"]["traversable"] is True


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


def test_robot_wheel_axes_match_the_rolled_wheel_link_frame():
    root = ET.parse(Path(__file__).parents[1] / "models" / "terralift_mecanum" / "model.sdf").getroot()
    axes = [joint.find("axis/xyz") for joint in root.findall(".//joint") if joint.attrib["name"].endswith("_joint")]
    assert len(axes) == 4
    assert all(axis is not None and axis.text.strip() == "0 0 -1" for axis in axes)


def test_static_map_matches_resized_physical_walls_and_scenario():
    """Catch a resized map/scenario drifting out of sync with Gazebo geometry."""
    root = Path(__file__).parents[1]
    generator = runpy.run_path(str(root / "scripts" / "generate_spatial_map.py"))
    regions = generator["FREE_REGIONS"]
    world = ET.parse(root / "worlds" / "spatial_warehouse.sdf").getroot()
    walls = []
    for model in world.findall("./world/model"):
        if model.attrib["name"] == "ground":
            continue
        for collision in model.findall("./link/collision"):
            sx, sy, _ = map(float, collision.findtext("geometry/box/size").split())
            x, y, *_ = map(float, collision.findtext("pose").split())
            walls.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))

    def inside(x, y, rectangle):
        left, right, bottom, top = rectangle
        return left < x < right and bottom < y < top

    # Sample cell interiors across the building, avoiding exact wall boundaries.
    for ix in range(168):
        for iy in range(128):
            x, y = -21 + (ix + 0.5) * 0.25, -16 + (iy + 0.5) * 0.25
            mapped_free = any(inside(x, y, region) for region in regions)
            physical_wall = any(inside(x, y, wall) for wall in walls)
            assert mapped_free != physical_wall, (x, y)

    subject = scenario()
    spawn = list(map(float, world.findtext("./world/include/pose").split()))
    assert spawn[:2] == [subject.robot_start.x, subject.robot_start.y]
    for passage in subject.passages.values():
        pose, barrier = passage.inspection_pose, passage.blocker_pose
        assert any(inside(pose.x, pose.y, region) for region in regions)
        assert abs((pose.x - barrier.x) ** 2 + (pose.y - barrier.y) ** 2 - 4.0) < 1e-6
    blocker = ET.parse(root / "models" / "passage_blocker" / "model.sdf")
    barrier_width = float(blocker.findtext(".//collision/geometry/box/size").split()[1])
    for left, right, bottom, top in regions[5:]:
        assert barrier_width > min(right - left, top - bottom)
