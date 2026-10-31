"""Anthropic tool-use client for high-level spatial verification only."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import json
import os
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional


class ClaudeProtocolError(RuntimeError):
    pass


TOOLS: List[Dict[str, Any]] = [
    {
        "name": "navigate_to",
        "description": "Navigate with Nav2 to a known room. This is a high-level request, never a motor command.",
        "input_schema": {"type": "object", "properties": {"location_id": {"type": "string"}}, "required": ["location_id"], "additionalProperties": False},
    },
    {
        "name": "inspect_passage",
        "description": "Navigate to a passage's inspection pose and return a proximity-limited LiDAR observation.",
        "input_schema": {"type": "object", "properties": {"passage_id": {"type": "string"}}, "required": ["passage_id"], "additionalProperties": False},
    },
    {
        "name": "inspect_location",
        "description": "Navigate to a room and report proximity-limited semantic marker observations.",
        "input_schema": {"type": "object", "properties": {"location_id": {"type": "string"}}, "required": ["location_id"], "additionalProperties": False},
    },
    {
        "name": "update_memory",
        "description": "Explicitly revise one believed passage or semantic entity using previously returned evidence IDs.",
        "input_schema": {
            "type": "object",
            "properties": {
                "updates": {"type": "array", "minItems": 1, "items": {"type": "object", "properties": {
                    "kind": {"type": "string", "enum": ["passage", "entity"]},
                    "id": {"type": "string"},
                    "traversable": {"type": "boolean"},
                    "room_id": {"type": "string"},
                }, "required": ["kind", "id"], "additionalProperties": False}},
                "evidence_ids": {"type": "array", "minItems": 1, "items": {"type": "string"}},
            },
            "required": ["updates", "evidence_ids"], "additionalProperties": False,
        },
    },
    {
        "name": "answer_question",
        "description": "Give the final concise answer only after sufficient evidence and explicit memory revision.",
        "input_schema": {
            "type": "object",
            "properties": {
                "answer": {"type": "string"}, "reason": {"type": "string"},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                "evidence_ids": {"type": "array", "items": {"type": "string"}},
            },
            "required": ["answer", "reason", "evidence_ids"], "additionalProperties": False,
        },
    },
]


SYSTEM_PROMPT = """You are the high-level spatial-belief agent in a mobile-robot experiment.
You receive only an old, structured map and physical observations returned by tools. Treat the map as a belief, not truth. Do not infer or claim hidden changes that were not observed. Choose targeted verification actions relevant to the question; Nav2, not you, controls motion.

When evidence contradicts a belief, call update_memory with the exact evidence IDs before answering. Then use your revised graph and semantic state to reason about the question. The question family semantic_distance_ranking means greatest shortest traversable-path cost from the A-flag room to a room containing a B flag; unreachable rooms are excluded. Return the room ID exactly in answer_question.answer, with a short explanation and evidence IDs. Never request hidden ground truth, obstacle locations, controller state, raw credentials, or internal reasoning traces."""


def _serialise(value: Any) -> Any:
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, list):
        return [_serialise(item) for item in value]
    if isinstance(value, dict):
        return {key: _serialise(item) for key, item in value.items()}
    return value


def parse_tool_blocks(content: Iterable[Any]) -> List[Dict[str, Any]]:
    """Return strictly validated tool calls from API-visible response blocks."""
    calls: List[Dict[str, Any]] = []
    for block in content:
        raw = _serialise(block)
        if isinstance(raw, dict) and raw.get("type") == "tool_use":
            name = raw.get("name")
            tool_input = raw.get("input")
            tool_id = raw.get("id")
            if name not in {item["name"] for item in TOOLS} or not isinstance(tool_input, dict) or not isinstance(tool_id, str):
                raise ClaudeProtocolError("Claude returned an invalid tool block")
            calls.append({"id": tool_id, "name": name, "input": tool_input})
    return calls


class ClaudeAgent:
    """Official Anthropic SDK client with durable request/response records."""

    def __init__(self, model: Optional[str], api_key: Optional[str], data: Any, transcript: Any, max_tokens: int = 1200, max_retries: int = 2) -> None:
        self.model = model or os.environ.get("ANTHROPIC_MODEL", "claude-haiku-5-5")
        self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
        if not self.api_key:
            raise RuntimeError("ANTHROPIC_API_KEY is required unless --mock is selected")
        try:
            from anthropic import Anthropic
        except ImportError as exc:  # pragma: no cover - depends on deployment
            raise RuntimeError("Install requirements-spatial-experiment.txt to use Claude") from exc
        self.client = Anthropic(api_key=self.api_key)
        self.data, self.transcript, self.max_tokens, self.max_retries = data, transcript, max_tokens, max_retries
        self.usage = {"api_calls": 0, "input_tokens": 0, "output_tokens": 0, "api_latency_sec": 0.0}

    def next(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        request_number = self.usage["api_calls"] + 1
        request = {"request_number": request_number, "model": self.model, "max_tokens": self.max_tokens, "system": SYSTEM_PROMPT, "tools": TOOLS, "messages": messages}
        start = time.monotonic()
        started_at = datetime.now(timezone.utc).isoformat()
        response = None
        for attempt in range(self.max_retries + 1):
            try:
                response = self.client.messages.create(model=self.model, max_tokens=self.max_tokens, system=SYSTEM_PROMPT, tools=TOOLS, messages=messages)
                break
            except Exception as exc:
                self.data.append("ai/requests_responses.jsonl", {
                    "request": request, "attempt": attempt + 1, "error": str(exc), "started_at": started_at,
                    "will_retry": attempt < self.max_retries,
                })
                if attempt >= self.max_retries:
                    raise
                time.sleep(0.75 * (2 ** attempt))
        if response is None:  # pragma: no cover - defensive
            raise ClaudeProtocolError("Claude response unexpectedly missing")
        latency = time.monotonic() - start
        raw_response = _serialise(response)
        usage = raw_response.get("usage", {}) if isinstance(raw_response, dict) else {}
        self.usage["api_calls"] += 1
        self.usage["input_tokens"] += int(usage.get("input_tokens", 0))
        self.usage["output_tokens"] += int(usage.get("output_tokens", 0))
        self.usage["api_latency_sec"] += latency
        self.data.append("ai/requests_responses.jsonl", {"request": request, "response": raw_response, "started_at": started_at, "latency_sec": latency})
        for block in raw_response.get("content", []):
            if block.get("type") == "text":
                self.transcript.line("CLAUDE", block.get("text", ""), request_number=request_number)
            elif block.get("type") == "tool_use":
                self.transcript.line("CLAUDE TOOL", f"{block.get('name')}({json.dumps(block.get('input', {}), sort_keys=True)})", request_number=request_number)
        return raw_response


class MockClaudeAgent:
    """Deterministic fixture for integration tests; never selected by default."""

    def __init__(self, data: Any, transcript: Any) -> None:
        self.data, self.transcript = data, transcript
        self.step = 0
        self.memory: Optional[Dict[str, Any]] = None
        self._observed_traversable: Optional[bool] = None
        self.usage = {"api_calls": 0, "input_tokens": 0, "output_tokens": 0, "api_latency_sec": 0.0}

    def next(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        self.step += 1
        self.usage["api_calls"] += 1
        if self.step == 1:
            initial = json.loads(str(messages[0]["content"]))
            self.memory = initial["spatial_memory"]
            content = [{"type": "tool_use", "id": "mock-inspect", "name": "inspect_passage", "input": {"passage_id": "A_E"}}]
        elif self.step == 2:
            tool_result = messages[-1].get("content", [])
            if not tool_result:
                raise ClaudeProtocolError("Mock fixture did not receive the passage observation")
            observation = json.loads(str(tool_result[0]["content"]))
            status = observation.get("observed_status")
            if status not in {"open", "blocked"}:
                raise ClaudeProtocolError("Mock fixture requires an observed passage state")
            self._observed_traversable = status == "open"
            content = [{"type": "tool_use", "id": "mock-revise", "name": "update_memory", "input": {"updates": [{"kind": "passage", "id": "A_E", "traversable": self._observed_traversable}], "evidence_ids": ["observation_001"]}}]
        else:
            if self.memory is None or self._observed_traversable is None:  # pragma: no cover - defensive
                raise ClaudeProtocolError("Mock agent was not initialized")
            self.memory["passages"]["A_E"]["traversable"] = self._observed_traversable
            answer = self._answer_from_memory()
            content = [{"type": "tool_use", "id": "mock-answer", "name": "answer_question", "input": {"answer": answer, "reason": "The observed passage state was incorporated into the graph before ranking the B-flag rooms.", "confidence": 1.0, "evidence_ids": ["observation_001"]}}]
        response = {"id": f"mock-{self.step}", "model": "mock", "content": content, "usage": {"input_tokens": 0, "output_tokens": 0}}
        self.data.append("ai/requests_responses.jsonl", {"request_number": self.step, "fixture": True, "messages": messages, "response": response})
        self.transcript.line("MOCK CLAUDE", json.dumps(content), request_number=self.step)
        return response

    def _answer_from_memory(self) -> str:
        """Fixture implementation of the defined graph question, not an answer constant."""
        import networkx as nx
        assert self.memory is not None
        graph = nx.Graph()
        graph.add_nodes_from(self.memory["rooms"])
        for passage_id, passage in self.memory["passages"].items():
            if passage["traversable"]:
                graph.add_edge(*passage["rooms"], weight=float(passage["cost"]), passage_id=passage_id)
        source = next(item["room_id"] for item in self.memory["entities"].values() if item["type"] == "A")
        candidates = sorted({item["room_id"] for item in self.memory["entities"].values() if item["type"] == "B"})
        reachable = [(float(nx.shortest_path_length(graph, source, room, weight="weight")), room) for room in candidates if nx.has_path(graph, source, room)]
        return max(reachable)[1]
