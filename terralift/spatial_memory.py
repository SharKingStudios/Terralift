"""Versioned, agent-visible spatial memory with append-only snapshots."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import copy
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from .spatial_models import Scenario


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def append_jsonl(path: Path, record: Mapping[str, Any]) -> None:
    import json
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, sort_keys=True) + "\n")
        stream.flush()


@dataclass
class SpatialMemory:
    scenario_id: str
    rooms: Dict[str, Dict[str, Any]]
    passages: Dict[str, Dict[str, Any]]
    entities: Dict[str, Dict[str, Any]]
    question: Dict[str, Any]
    version: int = 0
    evidence: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    @classmethod
    def from_scenario(cls, scenario: Scenario) -> "SpatialMemory":
        seed = scenario.to_agent_seed()
        return cls(
            # The internal scenario identifier is useful in on-disk memory
            # history, but must never appear in the agent-visible map: a
            # descriptive scenario filename can itself leak the manipulation.
            scenario_id=scenario.scenario_id,
            rooms=copy.deepcopy(seed["rooms"]),
            passages=copy.deepcopy(seed["passages"]),
            entities=copy.deepcopy(seed["entities"]),
            question=copy.deepcopy(seed["question"]),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "version": self.version,
            "rooms": copy.deepcopy(self.rooms),
            "passages": copy.deepcopy(self.passages),
            "entities": copy.deepcopy(self.entities),
            "question": copy.deepcopy(self.question),
            "evidence": copy.deepcopy(self.evidence),
        }

    def visible_context(self) -> Dict[str, Any]:
        """Explicit prompt allow-list; it protects against truth leakage."""
        context = self.to_dict()
        context.pop("scenario_id", None)
        return context

    def add_evidence(self, evidence: Mapping[str, Any]) -> str:
        evidence_id = str(evidence["evidence_id"])
        self.evidence[evidence_id] = copy.deepcopy(dict(evidence))
        return evidence_id

    def revise_passage(self, passage_id: str, traversable: bool, evidence_ids: Iterable[str]) -> Dict[str, Any]:
        if passage_id not in self.passages:
            raise ValueError(f"Unknown passage: {passage_id}")
        ids = [str(item) for item in evidence_ids]
        if not ids or any(item not in self.evidence for item in ids):
            raise ValueError("A passage revision requires recorded evidence")
        before = bool(self.passages[passage_id]["traversable"])
        self.passages[passage_id]["traversable"] = bool(traversable)
        self.passages[passage_id]["verification"] = "observed"
        self.passages[passage_id]["evidence_ids"] = ids
        self.version += 1
        return {"field": f"passages.{passage_id}.traversable", "before": before, "after": bool(traversable), "evidence_ids": ids}

    def revise_entity_room(self, entity_id: str, room_id: str, evidence_ids: Iterable[str]) -> Dict[str, Any]:
        if entity_id not in self.entities or room_id not in self.rooms:
            raise ValueError("Unknown entity or room")
        ids = [str(item) for item in evidence_ids]
        if not ids or any(item not in self.evidence for item in ids):
            raise ValueError("An entity revision requires recorded evidence")
        before = self.entities[entity_id]["room_id"]
        self.entities[entity_id]["room_id"] = room_id
        self.entities[entity_id]["verification"] = "observed"
        self.entities[entity_id]["evidence_ids"] = ids
        self.version += 1
        return {"field": f"entities.{entity_id}.room_id", "before": before, "after": room_id, "evidence_ids": ids}


class MemoryJournal:
    """Writes complete snapshots and granular modifications immediately."""

    def __init__(self, root: Path, memory: SpatialMemory) -> None:
        self.root = root
        self.memory = memory
        self.root.mkdir(parents=True, exist_ok=True)
        self.snapshots_path = root / "snapshots.jsonl"
        self.changes_path = root / "changes.jsonl"
        self.write_json(root / "initial_memory.json", memory.to_dict())
        self.snapshot("initial_memory")

    @staticmethod
    def write_json(path: Path, value: Mapping[str, Any]) -> None:
        import json
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
        temporary.replace(path)

    def snapshot(self, reason: str, simulation_time: Optional[float] = None) -> None:
        append_jsonl(self.snapshots_path, {
            "wall_time": utc_now(), "simulation_time": simulation_time,
            "reason": reason, "memory": self.memory.to_dict(),
        })

    def change(self, change: Mapping[str, Any], simulation_time: Optional[float] = None) -> None:
        append_jsonl(self.changes_path, {"wall_time": utc_now(), "simulation_time": simulation_time, **dict(change)})
        self.snapshot("memory_change", simulation_time)

    def finalize(self) -> None:
        self.write_json(self.root / "final_memory.json", self.memory.to_dict())
