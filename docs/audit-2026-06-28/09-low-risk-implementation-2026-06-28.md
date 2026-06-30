# Low-Risk Implementation Report

Date: 2026-06-28

This report records changes made after the audit without connecting to the
robot, launching ROS control, changing controller contracts, or rebuilding the
Unity player. It is an implementation record, not a machinery-safety claim.

## Completed

### Command and data hardening

- The browser debug client now emits `wristCommandEnabled: false` by default.
- Wrist control requires holding an explicit arm deadman. Pointer-up, pointer
  cancellation, lost pointer capture, keyboard release, window blur, page
  visibility loss, and disconnect all release it.
- The gateway fails closed for non-boolean deadman values while retaining the
  current legacy behavior for payloads that omit the field entirely.
- WebRTC pose positions and quaternions reject NaN and infinity before ROS
  publication. Oversized input payloads above 64 KiB are rejected.
- MoveIt-side quaternion helpers reject non-finite components.
- MoveIt-side quaternion normalization now uses one shared, overflow-safe math
  helper instead of three duplicated implementations.
- Arm targets and wrist TF positions reject non-finite components.
- Head and gripper trajectory values and durations have final finite checks
  before controller publication.
- The Servo bridge rejects invalid target/TF positions and checks the final
  linear and angular command before publication. A non-finite final command
  clears target state and publishes halt commands.
- Robot snapshot joint and TF data reject non-finite values, zero-length
  quaternions, stale gripper state, stale transforms, and future-skewed
  transforms.

These changes address the directly testable portions of VT-002, VT-003,
VT-010, VT-012, and M1-3. They do not implement protocol v2, authentication,
command ownership, or Servo TF-age handling.

### Repository and portfolio quality

- Added Apache-2.0 at the repository root, accurate ROS package metadata, and
  repository URLs.
- Added `.dockerignore` files and stopped ignoring all dotfiles and Unity's
  package lock.
- Added a documented `.env.example`; the Wi-Fi launcher validates DDS inputs
  and creates its generated CycloneDDS file at a private `mktemp` path.
- Marked the vendored SteamVR tree for GitHub Linguist so it does not dominate
  language statistics.
- Replaced the hard-coded Steam diagnostic at `test.sh` with a named diagnostic
  and a real project test command.
- Added static checks for Compose, maintained shell scripts, Python syntax,
  JSON, and ROS package XML.
- Added GitHub Actions jobs for static checks and ROS Python tests in a clean
  `ros:humble` container.
- Removed unmeasured low-latency wording and corrected configuration/path
  contradictions in the documentation.
- The browser debug server now copies only `index.html` into a private temporary
  web root. Requests for repository metadata and environment files return 404.

This completes or partially completes VT-007, VT-016, VT-018, VT-021, VT-024,
VT-032, VT-033, M0-3, M1-7, M2-1, M2-2, and M2-8.

## Verification performed

```text
Static checks:                   passed
ShellCheck:                      passed
MoveIt-side Python tests:        18 passed
Gateway/snapshot Python tests:   13 passed
Combined clean ros:humble run:   31 passed
Browser inline JavaScript parse: passed
Docker Compose base/Wi-Fi config: passed
Debug client /:                  HTTP 200, exact index.html content
Debug client /.git/config:       HTTP 404
Debug client /.env:              HTTP 404
Debug client /docker-compose.yml: HTTP 404
```

The tests mount the repository read-only into ROS containers. They do not start
ROS nodes, publish controller messages, join the robot network, or command the
robot.

## Deliberately deferred

The following changes require an interface decision, integration environment,
Unity validation, robot validation, credentials, or repository-owner action.
They were not treated as “low-hanging” changes:

- authenticated signaling and a single-client command lease;
- server-owned release on channel loss, which must be tied to ownership so one
  client cannot release another client's command;
- strict protocol v2 with schema version, client/session identity, sequence,
  timestamps, and removal of the legacy active-by-default fallback;
- command QoS changes, because reliability/depth/lifespan must be measured on
  the deployed networks;
- Servo dynamic-TF age policy and frozen-TF integration testing;
- Cartesian limits, acceleration/jerk limiting, collision checking, and named
  safe/experimental profiles;
- renaming the installed `image_listener` ROS package or changing topics,
  frames, controllers, network topology, launch order, and Servo tuning;
- splitting the large Python, browser, and Unity components;
- removing bundled SteamVR content or generating the Unity package lock without
  opening and validating the project in its pinned Unity version;
- deleting committed binary crash/bytecode artifacts and the autosave scene as
  part of a reviewed repository-history cleanup;
- rosbag2 recorder/runtime implementation, replay fixtures, plots, benchmark
  thresholds, and base driving;
- merging `qa` into the public default branch, branch protection, releases,
  secrets, and GitHub repository settings.

Before hardware testing, address the command-ownership design and select a
conservative, bounded motion profile. Then follow the audit's staged hardware
acceptance checklist with a physical emergency stop and a second observer.
