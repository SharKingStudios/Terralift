"""Focused tests that do not require ROS, Gazebo, or Anthropic credentials."""

from __future__ import annotations

import json
from pathlib import Path

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
