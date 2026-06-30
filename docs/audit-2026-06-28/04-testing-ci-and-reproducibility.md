# Testing, CI and Reproducibility Plan

## Current state

The current 12 MoveIt-side and 3 gateway-side tests pass in local ROS images.
They test controller delta mapping, quaternion error, feed-forward reset,
deadman release, halt messages and gateway deadman parsing. This is a useful
foundation, especially because deadman behavior is tested directly.

Missing infrastructure:

- no CI workflow;
- no package test dependencies or ament lint setup;
- no coverage report;
- no WebRTC/HTTP, snapshot, head or gripper tests;
- no launch/integration/system tests;
- no JavaScript or Unity tests;
- no deterministic replay fixture;
- no measured physical acceptance suite.

## Test pyramid

### Tier 1 — pure unit tests on every pull request

Extract ROS/Unity-independent math and state machines. Test:

- finite validation and normalization;
- quaternion/frame conversions with golden vectors;
- clutch anchor/delta mapping;
- workspace and velocity/acceleration limiting;
- head pan/tilt conversion, signs, limits and singular orientations;
- gripper normalization, deadband and duration;
- arm/base gate state machines for every event ordering;
- protocol schema, strict booleans, sequence gaps and stale timestamps;
- rosbag episode/window state transitions;
- browser interpolation and payload builders;
- Unity wrist workspace and gripper gesture mapping.

Use parameterized/property tests for NaN, infinity, extreme values and random
unit quaternions.

### Tier 2 — ROS node and launch tests

Run in a ROS Humble container:

- publishers/subscribers with the production QoS;
- fake TF and joint-state publishers;
- target timeout and stale-TF halt;
- no output while inactive;
- source/lease arbitration;
- gripper state freshness;
- Servo start/switch service failure and retry;
- launch argument/profile validation;
- shutdown while active publishes the expected halt behavior.

Use `launch_testing` where process lifecycle matters. Add a fake controller sink
that fails on non-finite or out-of-bound commands.

### Tier 3 — gateway/WebRTC integration

Start aiohttp on an ephemeral port and test:

- config, health, readiness and authentication;
- valid/invalid SDP and request limits;
- peer cleanup at each failure stage;
- data-channel schema and lease lifecycle;
- two competing clients;
- close, disconnect and timeout release;
- camera frame delivery and stale-camera readiness;
- backpressure and sequence-gap metrics.

### Tier 4 — rosbag replay/system tests

Create small deterministic MCAP fixtures with camera, joint state, TF, gates and
targets. Replay into a no-output profile and compare status/commands with golden
results. Validate bags through a reader, not only `ros2 bag info`.

### Tier 5 — Unity tests

EditMode:

- coordinate transforms and calibration;
- command gate state machine;
- DTO/schema serialization;
- gripper gesture mapping;
- URL/config resolution.

PlayMode:

- fake config/state/signaling server;
- connect/cancel/retry/dispose;
- data-channel open/close behavior;
- video track lifecycle;
- no command activation on reconnect.

### Tier 6 — hardware-in-the-loop acceptance

Run a versioned checklist with an operator and physical E-stop:

- startup with every dependency missing in turn;
- normal head, arm and gripper motion;
- deadman release and network removal;
- controller/headset tracking loss;
- stale TF/joint state;
- workspace/singularity/collision boundary;
- long-duration run and reconnect;
- disk-full recorder failure;
- base/arm interaction when driving is added.

## Proposed CI jobs

### `quality` (fast)

- Markdown link/style check;
- Ruff format/check or Black + Flake8;
- mypy/pyright on extracted pure modules;
- ShellCheck and shfmt;
- JSON/YAML/XML validation;
- PlantUML render check;
- secret scan and forbidden-artifact check.

### `ros-humble-test`

- build both packages from a clean ROS Humble image;
- `rosdep install` from package metadata;
- `colcon test` and `colcon test-result --verbose`;
- coverage/JUnit artifacts;
- launch and fake-graph integration tests.

### `container`

- `docker compose config --quiet` for every profile;
- build with fresh base images;
- image vulnerability scan;
- start services with fake ROS publishers;
- readiness and shutdown smoke test.

### `web`

- ESLint/Prettier;
- protocol/state tests in a headless browser;
- verify sensitive repository paths are not served.

### `unity`

Use a licensed/self-hosted runner only if available. Run EditMode tests on pull
requests touching Unity code and a Linux player build on protected merges.

## Reproducibility changes

1. Commit `Packages/packages-lock.json`.
2. Pin container and external image identities; record digest updates in PRs.
3. Add `.dockerignore` files and immutable runtime Compose profiles.
4. Store Git SHA, dirty flag, image IDs, Unity version, parameter dump and config
   hashes in each run manifest and rosbag session.
5. Provide one command for tests and one for a fake/replay demonstration.
6. Add a simulation or no-output replay mode that does not require TIAGo/Vive.
7. Never commit large production bags; publish selected datasets as release or
   dataset artifacts with checksums and licenses.

## Performance benchmark

Use rosbag/source/receive/controller timestamps to generate:

- input and command rate histograms;
- source-to-gateway, gateway-to-Servo and command-to-state latency;
- target versus measured wrist position/orientation error;
- stop-latency CDF;
- video age/frame-rate/drop statistics;
- CPU, memory and network throughput;
- clipping, rejection and stale-state counts.

Add a `tools/analysis/` command that takes one session directory and produces a
machine-readable summary plus portfolio-ready plots. Define thresholds in a
checked-in benchmark profile so regressions fail CI or nightly tests.
