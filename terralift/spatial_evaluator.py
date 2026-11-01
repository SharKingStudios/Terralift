"""Independent, deterministic scoring for recorded spatial trials.

The evaluator runs only after a trial terminates. It may use the hidden
scenario state for scoring, but no evaluator output is supplied to the agent.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, Mapping, Optional

from .spatial_models import Pose2D, Scenario


def _normalise_answer(value: Any) -> Any:
    if isinstance(value, str):
        return value.strip().upper()
    if isinstance(value, list):
        return sorted(_normalise_answer(item) for item in value)
    return value


def _observation_kind(observation: Mapping[str, Any]) -> Optional[str]:
    """Retain compatibility with older passage records lacking ``kind``."""
    kind = observation.get("kind")
    if kind in {"passage", "location"}:
        return str(kind)
    return "passage" if "passage_id" in observation else None


def _passage_observation_correct(observation: Mapping[str, Any], modified_state: Mapping[str, Any]) -> Optional[bool]:
    passage_id = observation.get("passage_id")
    if passage_id not in modified_state["passages"]:
        return None
    observed = observation.get("observed_status")
    if observed not in {"open", "blocked"}:
        return None
    expected = "open" if modified_state["passages"][passage_id]["traversable"] else "blocked"
    return observed == expected


def _location_observation_correct(observation: Mapping[str, Any], scenario: Scenario,
                                  modified_state: Mapping[str, Any]) -> Optional[bool]:
    """Check the simulated marker sensor against the recorded local pose."""
    if observation.get("kind") != "location" or "robot_pose" not in observation:
        return None
    try:
        robot_pose = Pose2D.from_value(observation["robot_pose"])
        marker_range = float(observation["range_m"])
    except (KeyError, TypeError, ValueError):
        return False
    expected = []
    for entity_id, entity in modified_state["entities"].items():
        entity_pose = Pose2D.from_value(entity["pose"])
        if math.hypot(robot_pose.x - entity_pose.x, robot_pose.y - entity_pose.y) <= marker_range:
            expected.append({"entity_id": entity_id, "type": entity["type"], "room_id": entity["room_id"]})
    actual = observation.get("visible_entities")
    if not isinstance(actual, list):
        return False
    sort_key = lambda item: item.get("entity_id", "")
    return sorted(actual, key=sort_key) == sorted(expected, key=sort_key)


def _entity_change_observed(component_id: str, observation: Mapping[str, Any], scenario: Scenario) -> bool:
    """A move is locally observed at either the prior or new room."""
    if observation.get("kind") != "location":
        return False
    original_room = scenario.initial_state()["entities"][component_id]["room_id"]
    modified_room = scenario.modified_state()["entities"][component_id]["room_id"]
    visible = {item.get("entity_id") for item in observation.get("visible_entities", []) if isinstance(item, Mapping)}
    if observation.get("room_id") == modified_room and component_id in visible:
        return True
    return observation.get("room_id") == original_room and component_id not in visible


def _memory_state(final_memory: Mapping[str, Any], scenario: Scenario) -> Dict[str, Any]:
    return {
        "passages": {
            key: {"traversable": value["traversable"], "cost": value.get("cost", scenario.passages[key].cost)}
            for key, value in final_memory["passages"].items()
        },
        "entities": {
            key: {"type": value["type"], "room_id": value["room_id"]}
            for key, value in final_memory["entities"].items()
        },
    }


def evaluate_trial(
    scenario: Scenario,
    final_memory: Mapping[str, Any],
    final_answer: Optional[Mapping[str, Any]],
    evidence: Iterable[Mapping[str, Any]],
    counters: Mapping[str, Any],
    status: str,
) -> Dict[str, Any]:
    """Score answer, belief, observations, coverage, cost, and API use separately."""
    original_state = scenario.initial_state()
    modified_state = scenario.modified_state()
    original = scenario.answer_for_state(original_state)
    expected = scenario.answer_for_state(modified_state)
    memory_state = _memory_state(final_memory, scenario)
    memory_answer = scenario.answer_for_state(memory_state)
    supplied_answer = None if final_answer is None else final_answer.get("answer")
    expected_value = _normalise_answer(expected["answer"])
    supplied_value = _normalise_answer(supplied_answer)
    memory_value = _normalise_answer(memory_answer["answer"])
    evidence_list = list(evidence)

    passage_matches = sum(
        bool(final_memory["passages"][passage_id]["traversable"]) == bool(truth["traversable"])
        for passage_id, truth in modified_state["passages"].items()
    )
    entity_matches = sum(
        final_memory["entities"][entity_id]["room_id"] == truth["room_id"]
        for entity_id, truth in modified_state["entities"].items()
    )
    passage_accuracy = passage_matches / max(1, len(modified_state["passages"]))
    entity_accuracy = entity_matches / max(1, len(modified_state["entities"]))
    memory_accuracy = (passage_matches + entity_matches) / max(
        1, len(modified_state["passages"]) + len(modified_state["entities"])
    )

    observation_results = []
    for observation in evidence_list:
        if _observation_kind(observation) == "passage":
            correct = _passage_observation_correct(observation, modified_state)
        elif observation.get("kind") == "location":
            correct = _location_observation_correct(observation, scenario, modified_state)
        else:
            correct = None
        if correct is not None:
            observation_results.append(bool(correct))

    changed_components = scenario.changed_components()
    observed_changed_components = []
    for component in changed_components:
        if component["kind"] == "passage":
            seen = any(
                _observation_kind(observation) == "passage"
                and observation.get("passage_id") == component["id"]
                and _passage_observation_correct(observation, modified_state) is True
                for observation in evidence_list
            )
        else:
            seen = any(_entity_change_observed(component["id"], observation, scenario) for observation in evidence_list)
        if seen:
            observed_changed_components.append(component)

    revised_changed_components = []
    for component in changed_components:
        if component["kind"] == "passage":
            correct = bool(final_memory["passages"][component["id"]]["traversable"]) == bool(modified_state["passages"][component["id"]]["traversable"])
        else:
            correct = final_memory["entities"][component["id"]]["room_id"] == modified_state["entities"][component["id"]]["room_id"]
        if correct:
            revised_changed_components.append(component)

    cited_ids = set() if final_answer is None else {str(item) for item in final_answer.get("evidence_ids", [])}
    evidence_by_id = {str(item.get("evidence_id")): item for item in evidence_list}
    cited_evidence_is_accurate = bool(cited_ids) and all(
        evidence_id in evidence_by_id
        and (
            _passage_observation_correct(evidence_by_id[evidence_id], modified_state) is True
            if _observation_kind(evidence_by_id[evidence_id]) == "passage"
            else _location_observation_correct(evidence_by_id[evidence_id], scenario, modified_state) is True
            if _observation_kind(evidence_by_id[evidence_id]) == "location"
            else False
        )
        for evidence_id in cited_ids
    )
    # Correct answers and adequate evidence remain intentionally independent.
    coverage = None if not changed_components else len(observed_changed_components) / len(changed_components)
    revision_accuracy = None if not changed_components else len(revised_changed_components) / len(changed_components)
    # A true answer must not be allowed to masquerade as evidence collection.
    # For a changed condition, sufficient support requires a complete local
    # check of changed components plus accurate cited observations. For an
    # unchanged control, this deliberately remains false: a selective check
    # cannot establish that no uninspected change occurred.
    answer_evidence_sufficient = bool(changed_components) and coverage == 1.0 and cited_evidence_is_accurate

    return {
        "status": status,
        "scenario_control": scenario.study_control,
        "question_id": scenario.question.question_id,
        "question_family": scenario.question.family,
        "question_text": scenario.question.text,
        "original_correct_answer": original["answer"],
        "modified_correct_answer": expected["answer"],
        "answer_changed": _normalise_answer(original["answer"]) != expected_value,
        "final_answer": supplied_answer,
        "final_answer_correct": supplied_value == expected_value,
        "memory_derived_answer": memory_answer["answer"],
        "belief_answer_consistency": supplied_value == memory_value,
        "answer_consistent_with_final_memory": supplied_value == memory_value,
        "memory_passage_accuracy": passage_accuracy,
        "memory_entity_accuracy": entity_accuracy,
        "memory_accuracy": memory_accuracy,
        "graph_edge_accuracy": passage_accuracy,
        "observations_evaluable": len(observation_results),
        "observations_correct": sum(observation_results),
        "observation_accuracy": None if not observation_results else sum(observation_results) / len(observation_results),
        "changed_components": changed_components,
        "changed_components_observed": observed_changed_components,
        "verification_coverage": coverage,
        "changed_components_revised_correctly": revised_changed_components,
        "change_revision_accuracy": revision_accuracy,
        "change_detected": bool(observed_changed_components),
        "changed_edge_revised_correctly": all(
            component in revised_changed_components for component in changed_components if component["kind"] == "passage"
        ),
        "final_answer_cites_evidence": bool(cited_ids),
        "cited_evidence_is_observation_accurate": cited_evidence_is_accurate,
        "answer_supported_by_observed_evidence": answer_evidence_sufficient,
        "exploration_cost": {
            "inspection_actions": int(counters.get("inspection_actions", 0)),
            "navigation_actions": int(counters.get("navigation_actions", 0)),
            "travel_distance_m": float(counters.get("travel_distance_m", 0.0)),
            "simulated_movement_time_sec": counters.get("simulated_movement_time_sec"),
            "navigation_failures": int(counters.get("navigation_failures", 0)),
        },
        "api_usage": {
            "api_calls": int(counters.get("api_calls", 0)),
            "input_tokens": int(counters.get("input_tokens", 0)),
            "output_tokens": int(counters.get("output_tokens", 0)),
            "cache_creation_input_tokens": int(counters.get("cache_creation_input_tokens", 0)),
            "cache_read_input_tokens": int(counters.get("cache_read_input_tokens", 0)),
            "api_latency_sec": float(counters.get("api_latency_sec", 0.0)),
        },
        # Retained flat values preserve existing analysis scripts.
        "inspection_actions": int(counters.get("inspection_actions", 0)),
        "navigation_actions": int(counters.get("navigation_actions", 0)),
        "travel_distance_m": float(counters.get("travel_distance_m", 0.0)),
        "simulated_movement_time_sec": counters.get("simulated_movement_time_sec"),
        "api_calls": int(counters.get("api_calls", 0)),
        "input_tokens": int(counters.get("input_tokens", 0)),
        "output_tokens": int(counters.get("output_tokens", 0)),
        "cache_creation_input_tokens": int(counters.get("cache_creation_input_tokens", 0)),
        "cache_read_input_tokens": int(counters.get("cache_read_input_tokens", 0)),
        "api_latency_sec": float(counters.get("api_latency_sec", 0.0)),
        "wall_clock_runtime_sec": float(counters.get("wall_clock_runtime_sec", 0.0)),
        "navigation_failures": int(counters.get("navigation_failures", 0)),
        "reasoning_failure": None if supplied_value == expected_value else "answer_not_correct",
    }
