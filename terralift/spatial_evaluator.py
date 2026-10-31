"""Independent, deterministic scoring for recorded spatial trials."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Mapping, Optional

from .spatial_models import Scenario


def _normalise_answer(value: Any) -> Any:
    if isinstance(value, str):
        return value.strip().upper()
    if isinstance(value, list):
        return sorted(_normalise_answer(item) for item in value)
    return value


def evaluate_trial(
    scenario: Scenario,
    final_memory: Mapping[str, Any],
    final_answer: Optional[Mapping[str, Any]],
    evidence: Iterable[Mapping[str, Any]],
    counters: Mapping[str, Any],
    status: str,
) -> Dict[str, Any]:
    original = scenario.answer_for_state(scenario.initial_state())
    modified_state = scenario.modified_state()
    expected = scenario.answer_for_state(modified_state)
    memory_state = {
        "passages": {key: {"traversable": value["traversable"], "cost": value.get("cost", scenario.passages[key].cost)} for key, value in final_memory["passages"].items()},
        "entities": {key: {"type": value["type"], "room_id": value["room_id"]} for key, value in final_memory["entities"].items()},
    }
    memory_answer = scenario.answer_for_state(memory_state)
    supplied_answer = None if final_answer is None else final_answer.get("answer")
    changed_passages = [change["passage_id"] for change in scenario.hidden_changes if change["type"] in {"block_passage", "open_passage"}]
    edge_matches = 0
    for passage_id, value in modified_state["passages"].items():
        if bool(final_memory["passages"][passage_id]["traversable"]) == bool(value["traversable"]):
            edge_matches += 1
    evidence_list = list(evidence)
    changed_evidence = [
        item for item in evidence_list
        if item.get("passage_id") in changed_passages and item.get("observed_status") == "blocked"
    ]
    expected_value = _normalise_answer(expected["answer"])
    supplied_value = _normalise_answer(supplied_answer)
    memory_value = _normalise_answer(memory_answer["answer"])
    return {
        "status": status,
        "question_id": scenario.question.question_id,
        "question_family": scenario.question.family,
        "question_text": scenario.question.text,
        "original_correct_answer": original["answer"],
        "modified_correct_answer": expected["answer"],
        "answer_changed": _normalise_answer(original["answer"]) != expected_value,
        "final_answer": supplied_answer,
        "final_answer_correct": supplied_value == expected_value,
        "answer_consistent_with_final_memory": supplied_value == memory_value,
        "memory_derived_answer": memory_answer["answer"],
        "change_detected": bool(changed_evidence),
        "changed_edge_revised_correctly": all(
            bool(final_memory["passages"][passage_id]["traversable"])
            == bool(modified_state["passages"][passage_id]["traversable"])
            for passage_id in changed_passages
        ),
        "graph_edge_accuracy": edge_matches / max(1, len(modified_state["passages"])),
        "answer_supported_by_observed_evidence": bool(changed_evidence),
        "inspection_actions": int(counters.get("inspection_actions", 0)),
        "navigation_actions": int(counters.get("navigation_actions", 0)),
        "travel_distance_m": float(counters.get("travel_distance_m", 0.0)),
        "simulated_movement_time_sec": counters.get("simulated_movement_time_sec"),
        "api_calls": int(counters.get("api_calls", 0)),
        "input_tokens": int(counters.get("input_tokens", 0)),
        "output_tokens": int(counters.get("output_tokens", 0)),
        "api_latency_sec": float(counters.get("api_latency_sec", 0.0)),
        "wall_clock_runtime_sec": float(counters.get("wall_clock_runtime_sec", 0.0)),
        "navigation_failures": int(counters.get("navigation_failures", 0)),
        "reasoning_failure": None if supplied_value == expected_value else "answer_not_correct",
    }
