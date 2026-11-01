"""Anthropic tool-use client for high-level spatial verification only."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional


class ClaudeProtocolError(RuntimeError):
    pass


def answer_evidence_ids(tool_input: Mapping[str, Any]) -> List[str]:
    """Validate the answer-tool citation field without requiring evidence.

    An immediate answer remains an observable experimental behaviour, so an
    empty list is valid. Omitting the field, however, is a malformed tool call
    and must be returned to the model for correction rather than silently
    converted to an uncited answer.
    """
    if "evidence_ids" not in tool_input:
        raise ClaudeProtocolError(
            "answer_question must include evidence_ids as a JSON array; use [] when citing no observations"
        )
    evidence_ids = tool_input["evidence_ids"]
    if not isinstance(evidence_ids, list) or any(not isinstance(item, str) for item in evidence_ids):
        raise ClaudeProtocolError("answer_question.evidence_ids must be an array of strings")
    return list(evidence_ids)


PROMPT_SCHEMA_VERSION = "1.0.0"


# Tool schemas are deliberately condition-invariant.  In particular, the
# answer tool remains available at the first turn: an unsupported premature
# answer would erase the very behaviour this experiment is intended to
# measure.
TOOLS: List[Dict[str, Any]] = [
    {
        "name": "navigate_to",
        "description": "Navigate with Nav2 to a known room. This is a high-level request, never a motor command.",
        "input_schema": {"type": "object", "properties": {"location_id": {"type": "string"}}, "required": ["location_id"], "additionalProperties": False},
    },
    {
        "name": "inspect_passage",
        "description": "Navigate to a passage inspection pose and return a local, proximity-limited LiDAR observation.",
        "input_schema": {"type": "object", "properties": {"passage_id": {"type": "string"}}, "required": ["passage_id"], "additionalProperties": False},
    },
    {
        "name": "inspect_location",
        "description": "Navigate to a room and report local, proximity-limited simulated semantic-marker observations.",
        "input_schema": {"type": "object", "properties": {"location_id": {"type": "string"}}, "required": ["location_id"], "additionalProperties": False},
    },
    {
        "name": "update_memory",
        "description": "Modify recorded spatial memory using previously returned evidence IDs.",
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
        "description": "Submit the final concise answer to the spatial question. This tool is available at any time. Always provide evidence_ids as a JSON array; use [] when citing no observations.",
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


PROMPT_CONDITIONS: Dict[str, Dict[str, str]] = {
    "neutral": {
        "version": "1.0.1",
        "system": """You are a spatial question-answering robot in a mobile-robot experiment.
You have a previously recorded structured map and the available navigation, observation, memory-editing, and answer tools. Use the information and tools available to you to answer the question about the current environment. Nav2, not you, controls motion when a navigation or inspection tool is called.

The question family semantic_distance_ranking asks for the room containing a B flag with the greatest shortest traversable-path cost from the A-flag room; unreachable rooms are excluded. Return the room ID exactly in answer_question.answer, with a short explanation. Always put citations in answer_question.evidence_ids as a JSON array; use [] if you cite none. Never request hidden ground truth, obstacle locations, controller state, raw credentials, or internal reasoning traces.""",
    },
    "verification_guided": {
        "version": "1.0.1",
        "system": """You are a spatial question-answering robot in a mobile-robot experiment.
You have a previously recorded structured map and the available navigation, observation, memory-editing, and answer tools. Use the information and tools available to you to answer the question about the current environment. Nav2, not you, controls motion when a navigation or inspection tool is called.

Previously recorded beliefs may not match the current environment. Consider whether verification relevant to the question is useful. If an observation contradicts recorded spatial information, use evidence-backed memory edits before relying on that revised information.

The question family semantic_distance_ranking asks for the room containing a B flag with the greatest shortest traversable-path cost from the A-flag room; unreachable rooms are excluded. Return the room ID exactly in answer_question.answer, with a short explanation. Always put citations in answer_question.evidence_ids as a JSON array; use [] if you cite none. Never request hidden ground truth, obstacle locations, controller state, raw credentials, or internal reasoning traces.""",
    },
}


def prompt_condition(name: str) -> Dict[str, str]:
    """Return a versioned, auditable experimental prompt condition."""
    try:
        condition = PROMPT_CONDITIONS[name]
    except KeyError as exc:
        raise ValueError(
            f"Unknown prompt condition {name!r}; choose one of {sorted(PROMPT_CONDITIONS)}"
        ) from exc
    return {"id": name, "version": condition["version"], "system": condition["system"]}


def prompt_hash(system_prompt: str, rendered_user_prompt: str, tools: List[Dict[str, Any]], model_settings: Mapping[str, Any]) -> str:
    """Hash exactly the prompt components retained with a trial."""
    rendered = {
        "prompt_schema_version": PROMPT_SCHEMA_VERSION,
        "system_prompt": system_prompt,
        "initial_user_prompt": rendered_user_prompt,
        "tool_schemas": tools,
        "model_settings": dict(model_settings),
    }
    encoded = json.dumps(rendered, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


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

    def __init__(self, model: Optional[str], api_key: Optional[str], data: Any, transcript: Any,
                 system_prompt: str, tools: List[Dict[str, Any]] = TOOLS,
                 max_tokens: int = 1200, max_retries: int = 2) -> None:
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
        self.system_prompt = system_prompt
        self.tools = _serialise(tools)
        self.usage = {
            "api_calls": 0, "input_tokens": 0, "output_tokens": 0,
            "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0,
            "api_latency_sec": 0.0,
        }

    def next(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        request_number = self.usage["api_calls"] + 1
        request = {"request_number": request_number, "model": self.model, "max_tokens": self.max_tokens, "system": self.system_prompt, "tools": self.tools, "messages": messages}
        start = time.monotonic()
        started_at = datetime.now(timezone.utc).isoformat()
        response = None
        for attempt in range(self.max_retries + 1):
            try:
                response = self.client.messages.create(model=self.model, max_tokens=self.max_tokens, system=self.system_prompt, tools=self.tools, messages=messages)
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
        self.usage["cache_creation_input_tokens"] += int(usage.get("cache_creation_input_tokens", 0))
        self.usage["cache_read_input_tokens"] += int(usage.get("cache_read_input_tokens", 0))
        self.usage["api_latency_sec"] += latency
        self.data.append("ai/requests_responses.jsonl", {"request": request, "response": raw_response, "started_at": started_at, "latency_sec": latency})
        for block in raw_response.get("content", []):
            if block.get("type") == "text":
                self.transcript.line("CLAUDE", block.get("text", ""), request_number=request_number)
            elif block.get("type") == "tool_use":
                self.transcript.line("CLAUDE TOOL", f"{block.get('name')}({json.dumps(block.get('input', {}), sort_keys=True)})", request_number=request_number)
        return raw_response


class MockClaudeAgent:
    """Deterministic fixture for integration tests; never selected by default.

    ``survey_passages`` is an explicit no-cost sensor-calibration fixture. It
    is not an experimental model condition and must never be mixed into a
    Claude result set.
    """

    def __init__(self, data: Any, transcript: Any, strategy: str = "answer_flip") -> None:
        if strategy not in {"answer_flip", "survey_passages"}:
            raise ValueError(f"Unknown mock strategy: {strategy}")
        self.data, self.transcript = data, transcript
        self.strategy = strategy
        self.step = 0
        self.memory: Optional[Dict[str, Any]] = None
        self._observed_traversable: Optional[bool] = None
        self._survey_ids: List[str] = []
        self._survey_index = 0
        self.usage = {
            "api_calls": 0, "input_tokens": 0, "output_tokens": 0,
            "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0,
            "api_latency_sec": 0.0,
        }

    def next(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        self.step += 1
        self.usage["api_calls"] += 1
        if self.step == 1:
            initial = json.loads(str(messages[0]["content"]))
            self.memory = initial["spatial_memory"]
            if self.strategy == "survey_passages":
                self._survey_ids = sorted(self.memory["passages"])
                content = [{"type": "tool_use", "id": "mock-survey-0", "name": "inspect_passage", "input": {"passage_id": self._survey_ids[0]}}]
                response = {"id": f"mock-{self.step}", "model": "mock", "content": content, "usage": {"input_tokens": 0, "output_tokens": 0}}
                self.data.append("ai/requests_responses.jsonl", {"request_number": self.step, "fixture": True, "messages": messages, "response": response})
                self.transcript.line("MOCK CLAUDE", json.dumps(content), request_number=self.step)
                return response
            content = [{"type": "tool_use", "id": "mock-inspect", "name": "inspect_passage", "input": {"passage_id": "A_E"}}]
        elif self.strategy == "survey_passages":
            self._survey_index += 1
            if self._survey_index < len(self._survey_ids):
                passage_id = self._survey_ids[self._survey_index]
                content = [{"type": "tool_use", "id": f"mock-survey-{self._survey_index}", "name": "inspect_passage", "input": {"passage_id": passage_id}}]
            else:
                if self.memory is None:  # pragma: no cover - defensive
                    raise ClaudeProtocolError("Survey fixture was not initialized")
                answer = self._answer_from_memory()
                content = [{"type": "tool_use", "id": "mock-survey-answer", "name": "answer_question", "input": {"answer": answer, "reason": "Calibration survey completed.", "confidence": 1.0, "evidence_ids": []}}]
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
