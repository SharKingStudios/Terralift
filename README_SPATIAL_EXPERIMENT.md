# Claude-guided Gazebo spatial-belief experiment

This is an isolated ROS 2 Jazzy / Gazebo Harmonic demonstration for Terralift. It does not start the physical drivetrain, camera, LiDAR, or the existing hardware launch files.

The default `distance_ranking_flip` trial has five visible warehouse zones, `A`–`E`, arranged as an asymmetric enclosed loop. A recorded map contains one A flag in A and B flags in D and E; there is deliberately no B flag beside the robot's A-room spawn. A–E is initially shown as open, making D the farthest B-flag room. Before Claude receives any prompt, the controller inserts a bright physical barrier into A–E, making E the deterministic answer. Claude receives only the historical map and local tool observations—not the barrier, its changed graph, or the changed answer.

The current world has one A flag in A and B flags only in D and E. There is deliberately no B marker in A or beside the robot spawn. The red A--E passage is the dynamically blocked link in the default answer-flip scenario.

The primary prompt condition is `neutral`: it asks a spatial question-answering robot to answer about the current environment with its recorded map and available tools, but does **not** state that a change happened, tell it to inspect, or tell it to revise memory. `verification_guided` is a separately versioned intervention that adds those instructions. A premature or incorrect `answer_question` call remains valid experimental behavior and is recorded rather than blocked.

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
RViz also opens by default with the static map, live LiDAR, TF, and Nav2
paths. Pass `--no-rviz` to keep Gazebo without the RViz window.

Choose the guided ablation explicitly; the neutral condition is the default:

```bash
./scripts/run_spatial_trial.sh --prompt-condition verification_guided
```

### Important: use WSL Bash, not PowerShell, for `.sh` files

`./scripts/run_spatial_trial.sh` is a Bash program. It will not launch the
ROS/Gazebo environment when run directly in a Windows PowerShell terminal.
Choose either entry point:

```powershell
# Windows PowerShell, from the repository root. This wrapper enters WSL for you.
.\scripts\run_spatial_trial.ps1
```

```bash
# Ubuntu/WSL Bash terminal.
cd /mnt/c/Users/Logan/Documents/GitHub/Terralift
./scripts/run_spatial_trial.sh
```

If the PowerShell wrapper reports a missing distribution, use the exact name
shown by `wsl --list --verbose`, for example:

```powershell
.\scripts\run_spatial_trial.ps1 -Distro Ubuntu-24.04
```

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

## Fresh-machine / agent bootstrap

An agent can provision another Windows PC from a clean GitHub clone without
guessing ROS versions. Use **WSL 2 + Ubuntu 24.04** (not Ubuntu 22.04, WSL 1,
or a Windows Python environment). In an elevated Windows PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
wsl --shutdown
```

After restarting/opening Ubuntu and creating its Linux username, install Git
once and clone the repository into the Linux filesystem for the best
Gazebo/colcon performance (for example `~/src/Terralift`, rather than
`/mnt/c/...`). Then run:

```bash
sudo apt update && sudo apt install -y git
git clone <your-GitHub-repository-URL> ~/src/Terralift
cd ~/src/Terralift
./scripts/setup_spatial_wsl.sh
cp .env.example .env
# Add ANTHROPIC_API_KEY to .env.
./scripts/run_spatial_trial.sh --mock --close  # visible no-cost validation
./scripts/run_spatial_trial.sh                 # real Claude data collection
```

The bootstrap needs internet access and `sudo`; it installs the pinned
Ubuntu 24.04 ROS 2 Jazzy, Gazebo Harmonic, Nav2, ROS/Gazebo bridge, MCAP, and
build packages. The normal run script creates its own Python virtual
environment and installs the pinned Anthropic SDK automatically. Keep the API
key only in `.env`, never in an agent prompt, command history, commit, or log.
The launcher accepts both Windows (CRLF) and Linux (LF) `.env` line endings.
It defaults to `ROS_DOMAIN_ID=42`, isolating the experiment from stale ROS
sessions on the default domain. Override it in `.env` only when you
deliberately need a different isolated domain. A custom `GZ_PARTITION` is
available as an advanced opt-in, but should be host-validated first because
some WSL/Gazebo combinations do not bridge sensor traffic across it reliably.

For the visible GUI on Windows, use WSL 2 with current WSLg and a current GPU
driver. If a machine has no usable graphical acceleration, the same experiment
can still be collected with `--headless`, but GUI inspection is unavailable.

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

## Visual robot-motion tour (no AI/API calls)

The enclosed-room layout is a cycle of dedicated corridors:

```text
                   [C] ---------+
 [B] --------------+             |
  |                               [D]
 [A] -- red A--E -- [E] <---------+
```

Each corridor is physically walled from the others; another route becomes
visible only after the robot enters its endpoint room. A--B is vertical,
B--C is horizontal, C--D goes right then down, D--E goes down then left, and
the low A--E connection is red. Corridors are 2.4 m wide. A, B, and D are
7.4 x 7.2 m; E is 8.6 x 7.2 m; C is intentionally tall at 8.6 x 14.0 m.
The static map covers 48 x 40 m at the original 0.05 m resolution. Robot
dimensions are unchanged.

Nav2 uses the full 0.90 x 0.82 m chassis-and-wheel envelope with 0.10 m
padding in both costmaps. DWB checks the oriented footprint for collisions;
a moderate 0.90 m inflation field discourages wall hugging without the
overly strong avoidance costs used during tuning. Forward motion is capped
at 1.2 m/s, angular speed at 1.4 rad/s, and angular acceleration at 2.5 rad/s²
for tighter, quicker turns. The controller runs at 30 Hz.

Gazebo targets 2.0 times real time (200%) while retaining the 0.001 s physics
step. Actual speed depends on the host and is visible in Gazebo; change
`real_time_factor` in `worlds/spatial_warehouse.sdf` to `1.0` for a real-time
comparison. Faster simulation does not increase the robot's simulated speed.
Wheel odometry can still drift: a successful Nav2 tour alone does not prove
that the physical robot followed the entire route without contact.

To verify chassis translation, turns, Nav2 goal stops, and the room geometry
before collecting data, run this from WSL Bash:

```bash
./scripts/run_spatial_room_tour.sh
```

It starts in A, visits B, C, D, E, and returns to A. The robot pauses briefly
at every room and Gazebo remains open with the robot stopped in A when the
tour is done. It does not call Claude or require an API key.

Both Gazebo and RViz open for the tour. On some Jazzy/WSLg systems RViz writes
one `indexed_8bit_image` GLSL warning as its first occupancy-grid message; it
does not affect Gazebo, Nav2, LiDAR, TF, or the remaining RViz displays.

Useful options:

```bash
# Explicit real model choice
./scripts/run_spatial_trial.sh --model claude-haiku-5-5 --gui

# Primary autonomous-verification condition (the default).
./scripts/run_spatial_trial.sh --prompt-condition neutral

# Guided-verification ablation, not the primary condition.
./scripts/run_spatial_trial.sh --prompt-condition verification_guided

# No GUI for a real trial
./scripts/run_spatial_trial.sh --headless

# Keep Gazebo but suppress the default RViz window
./scripts/run_spatial_trial.sh --gui --no-rviz

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

The full changed graph is never in the Claude system prompt, user message, tool result, debug line, or visible memory. The controller owns the hidden physical barrier and semantic fixture state; observations are returned only through the constrained tools below.

Claude has only these validated client-side tools:

### Observation fidelity

Passage inspection drives to the configured doorway pose and evaluates only a
0.22-radian cone of the actual bridged LiDAR scan toward the fixed doorway.
That avoids treating nearby corridor side walls as a blockage. The 2.0 m
threshold is calibrated for the 0.55 m forward scanner mount, the matched
0.20-radian Nav2/observation heading tolerance, and the spawned cross-corridor
collision barrier. The static
map deliberately remains open at that doorway, so the barrier reaches Nav2
only through live LiDAR.

Flag observations are range-limited *symbolic simulated marker* observations:
within 2.5 m of the robot, the controller returns the flag ID/type from the
scenario state. Flags are visible Gazebo fixtures and can be physically moved,
but this is not camera, AprilTag, or learned-vision detection. It is valid for
the language agent's navigation, evidence use, belief update, and graph
reasoning—not a benchmark of visual marker perception.

- `navigate_to(location_id)`
- `inspect_passage(passage_id)`
- `inspect_location(location_id)`
- `update_memory(updates, evidence_ids)`
- `answer_question(answer, evidence_ids)`

It cannot publish velocity commands. Memory changes must cite a recorded observation of the actual passage or entity; `answer_question` has no such prerequisite. The evaluator separately scores answer accuracy, full memory accuracy, belief-answer consistency, local-observation accuracy, changed-component verification coverage, revision accuracy, exploration cost, and API usage. A correct answer is never treated as proof that enough evidence was collected.

## Pre-batch controls and paired pilot

The available controls are validated on load against the true scenario graph:

| Scenario | Change type | Answer effect |
| --- | --- | --- |
| `distance_ranking_flip` | A–E physical closure | flips D → E |
| `topology_answer_invariant_bc_block` | B–C physical closure | remains D |
| `no_change_control` | none | remains D |
| `semantic_answer_flip_move_b_d_to_c` | B flag D → C | flips D → C |
| `semantic_answer_invariant_move_b_e_to_b` | non-farthest B flag E → B | remains D |

Run the small paired pilot before any matrix. It runs neutral and guided
conditions against the identical fixed label/seed for a topology-flip case,
an unchanged control, and a semantic answer-invariant case—six trials total.
It auto-closes each completed simulation and does not start any full batch:

```bash
./scripts/run_spatial_pilot.sh
```

Use `--headless` only when no display is available. The results root receives
`pilot_summary.json`; each individual trial also receives `audit.json`. Review
the saved API requests/responses and local-observation logs before proceeding.

## Results and replay

Each invocation allocates a new directory before Gazebo or Claude work begins:

```text
~/TerraliftExperiments/runs/2026-10-31_153000_distance_ranking_flip/
  manifest.json                 status.json             scenario.json
  ground_truth.json             events.jsonl            system.log
  bag/metadata.yaml             bag/*.mcap
  ai/transcript.txt             ai/transcript.jsonl
  ai/requests_responses.jsonl   ai/tool_results.jsonl    ai/api_usage.json
  ai/prompt_context.json        audit.json
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
| `spatial_evaluator.py` | Independent post-trial scoring against controller ground truth. |

The map generator intentionally excludes dynamic barriers from the static map. The Nav2 global/local obstacle layers receive `/scan`, so a spawned barrier changes live costmaps; after removal, normal clearing rays restore traversability.

The physical-passage result is derived from the live local LiDAR cone at the
recorded inspection pose. Semantic flags are physically visible and movable in
Gazebo, but their current interface is a range-gated symbolic simulated marker
sensor, not camera, AprilTag, or learned-vision perception. It supports a
controlled high-level semantic-belief condition only; do not report it as a
benchmark of visual semantic perception.

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
