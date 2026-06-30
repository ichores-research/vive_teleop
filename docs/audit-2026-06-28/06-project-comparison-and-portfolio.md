# Project Comparison and Portfolio Positioning

Comparisons are architectural, not claims that projects have identical scope or
hardware. Mature ecosystem projects have teams and years of work; peer projects
provide more relevant feature comparisons.

## Comparison matrix

| Project | Pattern worth adopting | Vive Teleop position |
| --- | --- | --- |
| ROS 2 `teleop_twist_joy` | Enable button required by default; configurable axes/scales; standard Twist boundary | Vive Teleop has a richer clutch and 6-DoF mapping, but the browser violates explicit-enable semantics. Reuse the enable-first model for driving. |
| Nav2 velocity smoother/collision monitor | Lifecycle, velocity/acceleration/deadband limits, timeout-to-zero, downstream collision monitoring | Needed for credible base driving; current arm bridge has timeout but disables its Cartesian caps and collision checking. |
| MoveIt Servo | Standard twist/joint interface, singularity handling, joint-limit scaling and collision checking | Good choice and correctly uses the seven-joint arm. Current configuration intentionally opts out of collision checking and needs profile-based justification. |
| rosbag2 | Whitelists, QoS overrides, pause/resume/snapshot, MCAP, lost-message statistics and direct reader tests | The design documents understand most of this, but implementation, validation and export are absent. |
| ROS 2 quality guidance/Nav2 repository | Package tests, ament lint, CI, pre-commit, license and system tests | Current tests are meaningful but manually run; automated quality checks are missing. |
| Unity WebRTC upstream | Dedicated samples, runtime/editor separation, extensive tests, CI metadata, license and third-party notices | The project uses the package appropriately but concentrates application behavior in one class and lacks Unity tests/lifecycle cleanup. |
| Quest2ROS2 | Dedicated bringup/custom-message structure, public demonstration, paper/citation and cross-robot framing | Vive Teleop has stronger WebRTC deployment and live-state anchoring; Quest2ROS2 has clearer research attribution and a reusable message boundary. |
| SpesRobotics `teleop` | Installable package, reusable robot adapters, physical and simulation examples, simple test command | Vive Teleop is richer and robot-integrated but harder for an outsider to install or exercise without exact hardware. |
| SO-101 ROS Physical AI | Separate ROS packages, locked environment, episode recording, bag conversion, visualization and policy inference | This is the best roadmap reference for turning the existing recording design into an end-to-end data story. Vive Teleop's VR/mobile-manipulator angle remains distinctive. |
| OpenArm ecosystem | Separate dataset/simulation tools, standardized evaluation environment and explicit licensing | Vive Teleop should separate recorder/export tools and add a reproducible simulated/replay evaluation environment. |
| PAL Robotics repositories | Robot-specific bringup/simulation packages and maintained TIAGo/PMB2 boundaries | Prefer upstream controller/simulation contracts and verify base topics against the installed robot stack. Do not duplicate robot description/controller ownership unnecessarily. |

## Where this repository is already stronger than many student projects

- Demonstrates a real mobile manipulator rather than only RViz/Gazebo.
- Solves bidirectional remote transport rather than assuming localhost ROS.
- Handles headset, wrist and gripper input together.
- Anchors commands to measured robot state to prevent initial jumps.
- Separates WebRTC signaling from ROS publishers/subscribers.
- Includes dual-interface CycloneDDS deployment and TURN traversal.
- Provides architecture diagrams and operational checks.
- Has real demo evidence and a nontrivial commit history.

## Where mature projects create a stronger impression

- Clean default branch and anonymous first-run experience.
- Explicit license, maintainers and release/version.
- Small packages with stable public interfaces.
- CI badges backed by actual repeatable jobs.
- Simulation/replay paths that reviewers can run.
- Quantitative benchmarks and documented limitations.
- Strict command authorization and safe defaults.
- Dataset recording connected to validation/export/visualization rather than a
  design-only folder.

## Portfolio changes with highest signal

1. Merge the polished state to the public default branch.
2. Fix P0 command/safety issues and publish the threat model.
3. Add CI with the 15 existing tests, then grow coverage.
4. Record one session and publish real tracking/latency/stop plots.
5. Add a replay/no-output demo that works without the robot.
6. Implement safe base driving and record a complete mobile-manipulation task.
7. Add one focused C++ component if targeting C++ robotics positions; the
   recorder controller or base guard would be technically justified choices.

## Suggested project claims

Good, defensible wording:

> Built and validated an end-to-end VR teleoperation prototype for a TIAGo
> mobile manipulator using ROS 2, MoveIt Servo, Unity, WebRTC and Docker.

> Implemented clutch-relative 6-DoF wrist control anchored to live TF, explicit
> deadman/timeouts, head and gripper control, and dual-network deployment.

Avoid until measured or implemented:

- “production safe”;
- “collision safe” while checking is disabled;
- “low latency” without a measurement and method;
- “ML dataset pipeline” while only design documents exist;
- “multi-user” while there is no command arbitration;
- “fully reproducible” while exact hardware and mutable dependencies are required.

## Interview preparation generated by this repository

Be ready to explain:

- Unity-to-robot coordinate and quaternion conversion;
- why clutch-relative control avoids activation jumps;
- why the robot and controller anchors both exist;
- target/TF/source timestamps and network jitter;
- MoveIt Servo command types, joint/singularity scaling and collision tradeoffs;
- why latest-value QoS differs from queued reliable commands;
- WebRTC data channels versus ROS bridge/WebSocket alternatives;
- ICE/TURN and the two-network topology;
- behavior under dropped input, stale TF and process restart;
- how rosbag observations/actions/outcomes become training samples;
- what makes a software deadman different from an emergency stop.

The candid answer about collision checking should be: it is currently disabled
for a supervised experimental configuration because observed proximity scaling
was too aggressive; that is a known limitation, not proof that collision checks
are unnecessary. Then describe the proposed safe/experimental profiles and the
measurements needed to tune it.
