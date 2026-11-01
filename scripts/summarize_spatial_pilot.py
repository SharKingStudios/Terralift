#!/usr/bin/env python3
"""Summarize completed paired pilot trials without modifying them."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results_directory", type=Path)
    args = parser.parse_args()
    rows = []
    for trial in sorted(args.results_directory.iterdir() if args.results_directory.exists() else []):
        manifest, metrics, audit = trial / "manifest.json", trial / "metrics.json", trial / "audit.json"
        if not (manifest.is_file() and metrics.is_file() and audit.is_file()):
            continue
        run, score, integrity = load(manifest), load(metrics), load(audit)
        rows.append({
            "trial_id": run["trial_id"], "scenario_id": run["scenario_id"], "seed": run.get("trial_seed"),
            "prompt_condition": run.get("prompt_condition", {}).get("id"),
            "status": score["status"], "answer_correct": score["final_answer_correct"],
            "memory_accuracy": score["memory_accuracy"], "verification_coverage": score["verification_coverage"],
            "inspection_actions": score["inspection_actions"], "travel_distance_m": score["travel_distance_m"],
            "api_calls": score["api_calls"], "hidden_state_leak_detected": integrity["hidden_state_leak_detected"],
            "forced_update_detected": integrity["controller_forced_memory_update_detected"],
            "immediate_answer": integrity["agent_answered_immediately"],
        })
    output = {"pilot_trials": rows, "trial_count": len(rows)}
    destination = args.results_directory / "pilot_summary.json"
    destination.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
