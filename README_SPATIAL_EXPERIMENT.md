# Claude-guided Gazebo spatial-belief experiment

This is an isolated ROS 2 Jazzy / Gazebo Harmonic demonstration for Terralift. It does not start the physical drivetrain, camera, LiDAR, or the existing hardware launch files.

The default `distance_ranking_flip` trial has five visible warehouse zones, `A`–`E`, arranged as a cycle. One A flag is in A; B flags are in D and E. In the historical memory, A–E is open, so D is the farthest B-flag room. Before Claude receives any prompt, the controller inserts a bright physical barrier into A–E. The new deterministic answer is E. Claude receives only the original memory and sensor observations—not the barrier, its changed graph, or the changed answer—and must inspect, revise its memory, and answer.

The robot is an included blue-and-orange Terralift-inspired four-wheel base with visibly mecanum-style wheels. For reliable Nav2 movement it uses Gazebo's maintained differential-drive system rather than unvalidated mecanum contact physics. This is deliberate: the experiment tests high-level belief revision, while preserving a physically visible robot, real collision geometry, LiDAR, `/clock`, odometry, Nav2 goals, and dynamic Gazebo entities.

## Start now in this WSL installation

The WSL Ubuntu 24.04 environment on this computer is already configured with
ROS 2 Jazzy, Gazebo Harmonic, Nav2, the ROS/Gazebo bridge, MCAP recording, and
the script-managed Python environment. From WSL, the only required setup is:

```bash
cd /mnt/c/Users/Logan/Documents/GitHub/Terralift
cp .env.example .env
# Edit .env and set ANTHROPIC_API_KEY.
./scripts/run_spatial_trial.sh
```

That starts the visible Gazebo trial with `claude-haiku-5-5` by default.

## Install on another Ubuntu 24.04 machine

Use a desktop session with a working OpenGL driver. Gazebo Harmonic is the supported Gazebo release for ROS 2 Jazzy.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces python3-pip python3-venv

cd /path/to/Terralift
```

On its first launch, the script creates `.venv-spatial-experiment` and installs the pinned official `anthropic` Python SDK, NetworkX, and `python-dotenv` automatically. No OpenAI API or SDK is used.

For WSL2, use current WSLg with GPU/OpenGL support, or use `--headless` and run the GUI from a native Ubuntu desktop. On a remote non-graphical Linux machine, use `--headless`; the live transcript remains on stdout and disk but no second graphical terminal can open.

## Configure Claude

```bash
cd /path/to/Terralift
cp .env.example .env
chmod 600 .env
# Edit .env and set ANTHROPIC_API_KEY to your Anthropic API key.
```

At minimum `.env` contains:

```dotenv
ANTHROPIC_API_KEY=your_key_here
ANTHROPIC_MODEL=claude-haiku-5-5
```

`.env` is ignored by Git. The controller never writes API keys, authorization headers, or environment dumps to trial outputs.

## Run one real visual trial

```bash
./scripts/run_spatial_trial.sh
```

The command builds the package, regenerates a map from permanent warehouse geometry, starts Gazebo Harmonic and ROS/Gazebo bridges, launches Nav2, waits for `/clock`, opens a second terminal that tails the live transcript, starts an MCAP rosbag, inserts the hidden physical A–E barrier, and starts the real Claude tool-use loop.

Useful options:

```bash
# Explicit real model choice
./scripts/run_spatial_trial.sh --model claude-haiku-5-5 --gui

# No GUI for a real trial
./scripts/run_spatial_trial.sh --headless

# Deterministic no-cost end-to-end controller fixture. This is never the default.
./scripts/run_spatial_trial.sh --mock --headless --close

# Stop launch automatically instead of leaving Gazebo open for visual inspection.
./scripts/run_spatial_trial.sh --close

# Legacy direct passage scenario, retained as a second configuration.
./scripts/run_spatial_trial.sh --scenario blocked_passage_demo
```

The default GUI remains open after the trial so the robot, rooms, flags, and orange A–E blocker can be inspected. Press `Ctrl-C` in the launch terminal afterward. `--close` shuts the launch down automatically at completion.

## What Claude can and cannot see

`config/spatial_scenarios/distance_ranking_flip.json` is loaded twice in intentionally different forms:

- `SpatialMemory.from_scenario()` creates the original agent-visible graph, entities, costs, and question.
- `GazeboWorldManager` owns controller-only hidden changes and ground truth.

The full changed graph is never in the Claude system prompt, user message, tool result, debug line, or visible memory. Passage inspection first navigates to a fixed inspection pose using `NavigateToPose`, then interprets the nearest valid actual bridged `/scan` range. The documented 1.6 m threshold covers Nav2's inspection-pose tolerance and the LiDAR mount offset; evidence remains tied to the inspected passage and never reads controller truth. The barrier is a spawned Gazebo collision model. Flag observations use a documented proximity-limited symbolic marker sensor based on robot pose and physically represented flag entities; they are ready for semantic flag-relocation scenarios through `move_entity`.

Claude has only these validated client-side tools:

- `navigate_to(location_id)`
- `inspect_passage(passage_id)`
- `inspect_location(location_id)`
- `update_memory(updates, evidence_ids)`
- `answer_question(answer, evidence_ids)`

It cannot publish velocity commands. Memory changes must cite a recorded observation of the actual passage or entity. The evaluator separately scores change detection, belief revision, answer correctness, graph accuracy, memory/answer consistency, and evidence support.

## Results and replay

Each invocation allocates a new directory before Gazebo or Claude work begins:

```text
~/TerraliftExperiments/runs/2026-10-31_153000_distance_ranking_flip/
  manifest.json                 status.json             scenario.json
  ground_truth.json             events.jsonl            system.log
  bag/metadata.yaml             bag/*.mcap
  ai/transcript.txt             ai/transcript.jsonl
  ai/requests_responses.jsonl   ai/tool_results.jsonl    ai/api_usage.json
  memory/initial_memory.json    memory/snapshots.jsonl
  memory/changes.jsonl          memory/final_memory.json
  navigation/goals.jsonl        navigation/observations.jsonl
  navigation/trajectory.csv     metrics.json            trial_summary.json
```

Override the root with `TERRALIFT_RESULTS_DIR=/data/experiments/runs` or `--results-dir /data/experiments/runs`. The terminal prints the absolute selected directory immediately and again in the final summary.

Inspect results:

```bash
RUN="$HOME/TerraliftExperiments/runs/<run-directory>"
jq . "$RUN/trial_summary.json"
jq . "$RUN/metrics.json"
tail -f "$RUN/ai/transcript.txt"
ros2 bag info "$RUN/bag"
ros2 bag play "$RUN/bag"
```

The controller begins bag recording before applying the hidden change and validates both `metadata.yaml` and a nonempty MCAP on completion. A trial cannot be marked fully complete if bag validation fails. SIGINT, Nav2/API errors, and timeouts leave every partial output in place with an explicit status.

## Architecture

| Component | Responsibility |
| --- | --- |
| `worlds/spatial_warehouse.sdf` | Permanent warehouse walls, visible zones/flags, and a recognizable robot. |
| `GazeboWorldManager` | Uses supported `ros_gz_sim` create/remove/set-pose clients to spawn barriers or move flags. |
| `spatial_trial_runner` | Waits for systems, records bag/events, invokes Claude, dispatches Nav2 goals, and finalizes data. |
| `ProximityObservationAdapter` | Converts only robot pose plus LaserScan / nearby markers into observation evidence. |
| `SpatialMemory` + `MemoryJournal` | Versioned structured beliefs, snapshots after observations/decisions, and evidence-backed diffs. |
| `ClaudeAgent` | Official Anthropic SDK tool-use protocol with complete visible request/response persistence and token/latency accounting. |
| `spatial_evaluator.py` | Independent NetworkX scoring against controller ground truth. |

The map generator intentionally excludes dynamic barriers from the static map. The Nav2 global/local obstacle layers receive `/scan`, so a spawned barrier changes live costmaps; after removal, normal clearing rays restore traversability.

## Troubleshooting

- `Missing required command: gz`: install `ros-jazzy-ros-gz` and open a new shell after sourcing `/opt/ros/jazzy/setup.bash`.
- `Timed out waiting for bridged /clock`: run `gz sim ...` independently and verify no second simulator or ROS node publishes `/clock`. The bridge is deliberately one-way from Gazebo to ROS.
- `navigate_to_pose action server was unavailable`: inspect `ros2 node list`, then `ros2 lifecycle get /bt_navigator`. The launch waits for Nav2, but a bad graphics/sensor startup can still prevent it from activating.
- Empty or invalid bag: check disk space and `ros2 bag info <run>/bag`; MCAP requires the Jazzy rosbag MCAP plugin normally included with desktop ROS.
- No extra chat terminal: set `DISPLAY`, install `gnome-terminal`, `xfce4-terminal`, or `xterm`, or set `SPATIAL_CHAT_TERMINAL` to one of them. The exact same transcript is always on stdout and `ai/transcript.txt`.
- `Claude API call limit reached`: inspect `ai/requests_responses.jsonl`, increase the `max_api_calls` node parameter only after reviewing the tool loop, and do not replace the real agent with the mock fixture for data collection.

## Tests

The ROS-independent tests are deliberately useful before a GPU-equipped ROS host is available:

```bash
python3 -m pytest test/test_spatial_experiment.py -q
python3 scripts/generate_spatial_map.py --output /tmp/spatial_warehouse.pgm
```

They cover scenario parsing and answer flip, graph representation, memory snapshots, hidden-truth isolation, Anthropic tool parsing, unique result directories/partial data, and deterministic evaluation. Full GUI/Nav2/LiDAR/rosbag/real-Claude validation must be run on Ubuntu 24.04 with the dependencies and credentials above; the implementation does not claim those checks pass on a non-ROS host.
