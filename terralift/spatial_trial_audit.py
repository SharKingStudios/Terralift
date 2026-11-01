"""Post-trial integrity checks for leakage, forced updates, and sensor records."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping


FORBIDDEN_PROMPT_KEYS = {
    "hidden_changes", "modified_state", "modified_answer", "ground_truth",
    "blocker_pose", "study_control",
}


def _records(path: Path) -> List[Dict[str, Any]]:
    if not path.is_file():
        return []
    result = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            result.append(json.loads(line))
    return result


def _nested_keys(value: Any) -> Iterable[str]:
    if isinstance(value, Mapping):
        for key, child in value.items():
            yield str(key)
            yield from _nested_keys(child)
    elif isinstance(value, list):
        for child in value:
            yield from _nested_keys(child)


def audit_trial(run_directory: str | Path) -> Dict[str, Any]:
    """Audit controller-visible artifacts without exposing truth to Claude."""
    root = Path(run_directory)
    prompt_path = root / "ai" / "prompt_context.json"
    prompt = json.loads(prompt_path.read_text(encoding="utf-8")) if prompt_path.is_file() else {}
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    requests = _records(root / "ai" / "requests_responses.jsonl")
    tool_records = _records(root / "ai" / "tool_results.jsonl")
    observations = _records(root / "navigation" / "observations.jsonl")
    changes = _records(root / "memory" / "changes.jsonl")

    prompt_leak_keys = sorted(set(_nested_keys(prompt)).intersection(FORBIDDEN_PROMPT_KEYS))
    request_leak_keys = sorted({
        key for record in requests for key in _nested_keys(record.get("request", {}))
        if key in FORBIDDEN_PROMPT_KEYS
    })
    scenario_id = str(manifest.get("scenario_id", ""))
    rendered_prompt = json.dumps(prompt, sort_keys=True)
    rendered_requests = "\n".join(json.dumps(record.get("request", {}), sort_keys=True) for record in requests)
    scenario_identifier_leaked = bool(scenario_id) and (scenario_id in rendered_prompt or scenario_id in rendered_requests)
    evidence_ids = {str(item.get("evidence_id")) for item in observations}
    update_calls = [item for item in tool_records if item.get("tool_call", {}).get("name") == "update_memory"]
    answer_calls = [item for item in tool_records if item.get("tool_call", {}).get("name") == "answer_question"]
    missing_update_evidence = [
        item.get("tool_call", {}).get("input", {}).get("evidence_ids", [])
        for item in update_calls
        if not set(map(str, item.get("tool_call", {}).get("input", {}).get("evidence_ids", []))).issubset(evidence_ids)
    ]
    requested_update_count = sum(
        len(item.get("tool_call", {}).get("input", {}).get("updates", [])) for item in update_calls
    )
    # ``inspect_passage`` also records an explicit ``unobserved`` result when
    # Nav2 cannot safely reach the inspection pose.  That is an important
    # navigation outcome, but it is not a sensor observation.  Do not let a
    # correctly reported safety failure make the local-LiDAR audit fail.
    passage_sensor_records = [
        item for item in observations
        if item.get("kind") == "passage" and item.get("observed_status") in {"open", "blocked"}
    ]
    passage_navigation_failure_records = [
        item for item in observations
        if item.get("kind") == "passage"
        and item.get("observed_status") == "unobserved"
        and not item.get("navigation", {}).get("success", True)
    ]
    semantic_sensor_records = [item for item in observations if item.get("kind") == "location"]
    passage_local = all(
        item.get("source") == "nearby_lidar" and "robot_pose" in item and "nearest_lidar_range_m" in item
        for item in passage_sensor_records
    )
    semantic_local = all(
        item.get("source") == "proximity_marker_sensor" and "robot_pose" in item and "range_m" in item
        for item in semantic_sensor_records
    )
    immediate_answer = bool(answer_calls) and not observations
    return {
        "audit_version": "1.0.0",
        "prompt_context_present": bool(prompt),
        "prompt_hash": prompt.get("prompt_hash"),
        "prompt_hidden_state_keys": prompt_leak_keys,
        "request_hidden_state_keys": request_leak_keys,
        "scenario_identifier_leaked": scenario_identifier_leaked,
        "hidden_state_leak_detected": bool(prompt_leak_keys or request_leak_keys or scenario_identifier_leaked),
        "agent_requested_memory_updates": len(update_calls),
        "memory_changes_logged": len(changes),
        "memory_update_missing_observation_ids": missing_update_evidence,
        "controller_forced_memory_update_detected": len(changes) > requested_update_count,
        "agent_answered_immediately": immediate_answer,
        "artifacts_preserved_without_navigation": immediate_answer and (root / "memory" / "final_memory.json").is_file(),
        "passage_sensor_records": len(passage_sensor_records),
        "passage_sensor_records_are_local_lidar": passage_local,
        "passage_navigation_failure_records": len(passage_navigation_failure_records),
        "semantic_sensor_records": len(semantic_sensor_records),
        "semantic_sensor_records_are_local_range_gated": semantic_local,
        "scientific_caveats": [
            "Semantic marker observations are a range-gated simulator interface, not camera, AprilTag, or learned-vision perception."
        ] if semantic_sensor_records else [],
    }
