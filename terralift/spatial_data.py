"""Durable trial outputs, live transcript, and rosbag process management."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Dict, Iterable, Mapping, Optional


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class TrialData:
    """Creates a never-reused output directory and flushes every record."""

    def __init__(self, scenario_id: str, root: Optional[str] = None) -> None:
        base = Path(root or os.environ.get("TERRALIFT_RESULTS_DIR", "~/TerraliftExperiments/runs")).expanduser()
        stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        candidate = base / f"{stamp}_{scenario_id}"
        suffix = 1
        while candidate.exists():
            suffix += 1
            candidate = base / f"{stamp}_{scenario_id}_{suffix:02d}"
        self.root = candidate.resolve()
        self.root.mkdir(parents=True, exist_ok=False)
        self.ai_dir = self.root / "ai"
        self.memory_dir = self.root / "memory"
        self.navigation_dir = self.root / "navigation"
        self.bag_dir = self.root / "bag"
        # ros2 bag record creates the bag directory itself and fails if it
        # already exists, so do not pre-create ``bag_dir`` here.
        for directory in (self.ai_dir, self.memory_dir, self.navigation_dir):
            directory.mkdir()
        self.write_json("status.json", {"status": "initialized", "updated_at": utc_now()})

    def path(self, relative: str) -> Path:
        return self.root / relative

    def write_json(self, relative: str, value: Mapping[str, Any]) -> None:
        destination = self.path(relative)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(destination)

    def append(self, relative: str, value: Mapping[str, Any]) -> None:
        destination = self.path(relative)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(value, sort_keys=True, default=str) + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def status(self, status: str, **extra: Any) -> None:
        self.write_json("status.json", {"status": status, "updated_at": utc_now(), **extra})

    def event(self, name: str, **payload: Any) -> Dict[str, Any]:
        event = {"event": name, "wall_time": utc_now(), **payload}
        self.append("events.jsonl", event)
        return event


class LiveTranscript:
    """Human-readable transcript written to disk and emitted to stdout."""

    def __init__(self, data: TrialData) -> None:
        self.path = data.ai_dir / "transcript.txt"
        self.json_path = data.ai_dir / "transcript.jsonl"

    def line(self, channel: str, message: str, **record: Any) -> None:
        elapsed = datetime.now().strftime("%H:%M:%S")
        rendered = f"[{elapsed}] {channel}: {message}"
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
            stream.flush()
        with self.json_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"wall_time": utc_now(), "channel": channel, "message": message, **record}, default=str) + "\n")
            stream.flush()
        print(rendered, flush=True)


class RosbagRecorder:
    """Explicit-topic rosbag2 recorder which never reports an empty bag valid."""

    def __init__(self, data: TrialData) -> None:
        self.data = data
        self.process: Optional[subprocess.Popen[str]] = None
        self.output = data.bag_dir
        self.log = data.path("system.log")

    def start(self, topics: Iterable[str]) -> None:
        if self.process is not None:
            raise RuntimeError("rosbag recorder is already running")
        if shutil.which("ros2") is None:
            raise RuntimeError("ros2 executable was not found; cannot record mandatory rosbag")
        command = ["ros2", "bag", "record", "--storage", "mcap", "--output", str(self.output), *topics]
        logfile = self.log.open("a", encoding="utf-8")
        self.process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, text=True)
        # rosbag creates metadata only after it has initialized. Fail early if it exits.
        time.sleep(1.0)
        if self.process.poll() is not None:
            raise RuntimeError(f"rosbag exited while starting with code {self.process.returncode}")

    def stop(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5.0)
        self.process = None

    def validate(self) -> Dict[str, Any]:
        metadata = self.output / "metadata.yaml"
        mcap_files = list(self.output.glob("*.mcap"))
        valid = metadata.is_file() and any(item.stat().st_size > 0 for item in mcap_files)
        result: Dict[str, Any] = {
            "valid": valid,
            "metadata_exists": metadata.is_file(),
            "mcap_files": [str(item) for item in mcap_files],
        }
        if valid and shutil.which("ros2"):
            completed = subprocess.run(["ros2", "bag", "info", str(self.output)], capture_output=True, text=True, timeout=20)
            result["ros2_bag_info_exit_code"] = completed.returncode
            result["ros2_bag_info"] = completed.stdout if completed.returncode == 0 else completed.stderr
            result["valid"] = completed.returncode == 0
        return result


def open_live_terminal(transcript_path: Path, enabled: bool = True) -> bool:
    """Open a terminal that tails the transcript without putting data in shell code."""
    if not enabled or not os.environ.get("DISPLAY"):
        return False
    terminal = os.environ.get("SPATIAL_CHAT_TERMINAL")
    tail_command = ["tail", "-n", "+1", "-f", str(transcript_path)]
    candidates = ([terminal] if terminal else []) + ["gnome-terminal", "xfce4-terminal", "xterm"]
    for candidate in candidates:
        if not candidate or shutil.which(candidate) is None:
            continue
        if candidate == "gnome-terminal":
            subprocess.Popen([candidate, "--", *tail_command], start_new_session=True)
        elif candidate == "xfce4-terminal":
            subprocess.Popen([candidate, "-e", " ".join(tail_command)], start_new_session=True)
        else:
            subprocess.Popen([candidate, "-hold", "-e", *tail_command], start_new_session=True)
        return True
    return False
