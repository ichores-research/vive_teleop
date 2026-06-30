# Vive Teleop Engineering Audit

Audit date: 2026-06-28

The original review documents in this folder describe repository commit
`744a70a` on branch `qa`. They recommend changes without silently rewriting the
audited baseline. A dated implementation report records later low-risk fixes.

## Executive assessment

The project is technically strong for an internship portfolio. It integrates a
physical TIAGo, ROS 2 Humble, MoveIt Servo, TF, direct robot controllers, Unity,
SteamVR/OpenVR, WebRTC, TURN, CycloneDDS, Docker, and dual-network deployment.
The working arm and head videos materially improve its credibility. The code
also contains several good engineering decisions: live-state initialization,
clutch-relative arm targets, explicit release messages, stale-command timeouts,
latest-target semantics, quaternion normalization, workspace constraints,
structured launch configuration, runtime preflight checks, and focused unit
tests around the most important deadman behavior.

The repository is still a supervised research prototype, not a production-safe
teleoperation system. The most important gaps are command authorization and
ownership, non-finite input handling, intentionally unbounded bridge velocity,
disabled collision checking, stale-TF acceptance in the feedback loop, an
implicitly active browser arm stream, insufficient integration testing, and a
debug server that exposes the repository directory over Wi-Fi. These are not
reasons to discard the architecture. They are the next engineering layer.

### Ratings for the audited snapshot

| Dimension | Rating | Reason |
| --- | ---: | --- |
| Project ambition | 9/10 | Real robot, VR, manipulation, video and networking in one system. |
| Demonstrated functionality | 8/10 | Real videos and operational scripts; no published measurements. |
| Architecture | 7/10 | Sensible subsystem boundary, but command authority and protocol contracts are implicit. |
| Safety and security | 4/10 | Good deadman ideas, but unsafe defaults and unauthenticated command ingress remain. |
| Code maintainability | 6/10 | Clear local functions, but several very large files and duplicated transforms/utilities. |
| Testing | 5/10 | Fifteen passing unit tests; no CI, integration, replay, Unity or failure-injection suite. |
| Reproducibility | 5/10 | Docker and scripts help, but mutable images, ignored lockfile, hardware dependency and bind mounts weaken it. |
| Repository presentation | 6/10 | Strong `qa` README, but the public default branch is behind and generated artifacts are tracked. |
| Internship signal after P0/P1 work | 9/10 | Would show end-to-end robotics plus disciplined verification and measured performance. |

## Documents

1. [Prioritized findings](01-prioritized-findings.md) is the primary defect and
   recommendation register.
2. [Code readability and maintainability](02-code-readability-and-maintainability.md)
   reviews each first-party subsystem and large file.
3. [Safety, security and reliability](03-safety-security-reliability.md) defines
   the threat model and a safer command architecture.
4. [Testing, CI and reproducibility](04-testing-ci-and-reproducibility.md) proposes
   a layered verification strategy and measurable acceptance criteria.
5. [Rosbag2 and driving roadmap](05-rosbag2-and-driving-roadmap.md) turns the two
   proposed features into staged, testable work.
6. [Project comparison and portfolio positioning](06-project-comparison-and-portfolio.md)
   compares this repository with maintained upstream and peer projects.
7. [Implementation backlog](07-implementation-backlog.md) converts the review
   into ordered, issue-ready work packages.
8. [Reference sources](08-reference-sources.md) records the primary sources used
   for the comparison.
9. [Low-risk implementation report](09-low-risk-implementation-2026-06-28.md)
   records the fixes completed without robot or headset testing and the work
   deliberately deferred.

## Scope and method

The review covered all first-party Python, C#, JavaScript/HTML, shell, Docker,
Compose, ROS launch, ROS package metadata, YAML configuration, CycloneDDS and
TURN configuration. Unity scenes and project settings were inspected where
they affect runtime or repository behavior. The vendored SteamVR implementation
was not reviewed line by line; how it is vendored, configured and consumed was
reviewed. Generated PlantUML images were not reverse-engineered because their
source `.puml` files exist.

The review included:

- source inspection with line-numbered references;
- comparison of implementation defaults with documentation claims;
- repository history, branch, tracked-file and artifact inspection;
- Docker Compose configuration validation;
- execution of the current Python tests inside existing local ROS images;
- comparison against upstream ROS 2, MoveIt, Nav2, rosbag2, Unity WebRTC and
  public teleoperation projects.

Test result at audit time:

```text
moveit_server: 12 passed
webrtc_server: 3 passed
total:         15 passed
```

The tests required a ROS environment and were run in existing local images with
the current source mounted read-only. There is not yet a repository CI workflow
that reproduces this result from a clean checkout.

## Limitations

- No command was sent to a physical robot during this review.
- No Unity player was rebuilt or run with a headset.
- Network latency, tracking error, stop latency and video delay were not measured.
- A software review cannot certify machinery safety. Collision, stability,
  emergency-stop and site procedures require validation with the robot owner.
- Line references describe commit `744a70a` and will drift after refactoring.
- The `docs/data-recording/` tree was untracked at audit time. Its content was
  reviewed, but it will not exist for other clones until committed.

## Recommended reading order

For immediate work, read the P0 table in the findings document and then the
safety architecture. For portfolio improvement, read the comparison document
and the first two milestones of the backlog. For rosbag2 or base driving, read
the dedicated roadmap only after the P0 command-path work.
