#!/usr/bin/env python3
"""Write/read a human-readable integrity audit for one saved trial."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from terralift.spatial_trial_audit import audit_trial


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    result = audit_trial(args.run_directory)
    destination = args.run_directory / "audit.json"
    destination.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
