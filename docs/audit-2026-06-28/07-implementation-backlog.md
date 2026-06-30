# Implementation Backlog

Sizes: **S** is a focused change, **M** spans a subsystem, **L** is a milestone.
Robot-time estimates are intentionally separate because lab access is usually
the constraint.

## Milestone 0 — make the public repository truthful and reviewable

| ID | Size | Work | Definition of done |
| --- | --- | --- | --- |
| M0-1 | S | Merge `qa` to default branch through PR | Anonymous repository page shows demos/docs; branch protection enabled. |
| M0-2 | S | Commit or intentionally remove data-recording design | Clean worktree; docs index links resolve in a fresh clone. |
| M0-3 | S | Add root license and correct package metadata | License matches manifests; maintainer/name/description/version are real. |
| M0-4 | S | Remove tracked bytecode/crash/autosave/upgrade artifacts | Forbidden-artifact CI check passes. |
| M0-5 | M | Reduce vendored SteamVR/template content and commit Unity lock | Clean Unity checkout restores and builds; licenses retained. |

## Milestone 1 — close command-path P0 issues

| ID | Size | Dependencies | Work and acceptance |
| --- | --- | --- | --- |
| M1-1 | M | — | Versioned strict command schema; reject missing/invalid booleans and non-finite values; fuzz tests. |
| M1-2 | L | M1-1 | Authenticated single-client command lease; conflict/expiry/reconnect tests. |
| M1-3 | M | M1-1 | Browser hold-to-command; false on blur/up/unload; opening input stays inactive. |
| M1-4 | M | M1-2 | Server publishes release on channel/peer/lease loss; measured stop latency. |
| M1-5 | S | — | Servo TF age/future-skew guard; frozen-TF integration test. |
| M1-6 | M + robot | — | Conservative bounded/collision-aware profile; experimental opt-in; measured velocities. |
| M1-7 | S | — | Serve only dedicated web assets; sensitive-path HTTP tests. |
| M1-8 | M | M1-2 | Generated credentials, origin allowlist and documented network trust boundary. |

## Milestone 2 — establish a professional quality baseline

| ID | Size | Work | Definition of done |
| --- | --- | --- | --- |
| M2-1 | M | Add ROS Humble CI | Clean build, 15 current tests, JUnit and coverage on every PR. |
| M2-2 | M | Add lint/format/static checks | Python, shell, web, JSON/YAML/XML and Markdown checks enforced. |
| M2-3 | M | Add head, gripper, snapshot and invalid-input tests | Finite/freshness/limits and protocol edge cases covered. |
| M2-4 | M | Add fake ROS graph/launch tests | Startup, stale state, timeout, service failure and shutdown tested. |
| M2-5 | M | Add capability readiness and `/healthz`/`/readyz` | Last sample ages and stable reason codes exposed. |
| M2-6 | M | Define command QoS and status message | Queue depth/lifespan documented and integration-tested; status recordable. |
| M2-7 | M | Create immutable runtime container profile | Pinned images, non-root where possible, health checks, no source bind mount. |
| M2-8 | S | Replace `test.sh` with `scripts/test.sh` and `scripts/doctor.sh` | Names reflect behavior; one command reproduces checks. |

## Milestone 3 — refactor for testable ownership

| ID | Size | Work |
| --- | --- | --- |
| M3-1 | L | Extract Python teleop math and arm state machine from the mixin/node. |
| M3-2 | L | Split Unity coordinator, peer clients, input sources, mapper, recorder and protocol DTO. |
| M3-3 | M | Split browser CSS/JS and add protocol/state tests. |
| M3-4 | M | Separate head and gripper controllers; add state freshness. |
| M3-5 | M | Create project-specific SteamVR action manifest. |
| M3-6 | M | Add shared golden transform vectors across Python/C#/JavaScript. |

## Milestone 4 — rosbag2 observability

| ID | Size | Dependencies | Work |
| --- | --- | --- | --- |
| M4-1 | S + robot | M2-6 | Live topic/type/QoS/rate/storage inventory. |
| M4-2 | M | M4-1 | Continuous MCAP recorder service, whitelist, volume, manifest, graceful stop. |
| M4-3 | M | M4-2 | Direct reader validator and small synthetic fixture. |
| M4-4 | L | M4-2 | Analysis tool for latency, jitter, tracking error and stop latency. |
| M4-5 | L | M4-3 | Segment/episode controller, events, post-roll, fragment recovery. |
| M4-6 | L | M4-5 | Offline dataset export and episode visualization. |

## Milestone 5 — safe base driving

| ID | Size | Dependencies | Work |
| --- | --- | --- | --- |
| M5-1 | S + robot | M2-6 | Verify TIAGo base controller, watchdog, odometry and obstacle topics. |
| M5-2 | M | M1-1, M3-5 | Left-controller base protocol and independent physical deadman. |
| M5-3 | L | M1-2, M5-1 | Base guard with lease, finite/freshness checks, limits, smoothing and timeout. |
| M5-4 | L + robot | M5-3 | Collision monitor and arm-extension speed/interlock validation. |
| M5-5 | M | M4-2, M5-3 | Record intent/effective command/odom/safety events. |
| M5-6 | M + robot | M5-4, M5-5 | Whole-task demo and measured stop/trajectory results. |

## Milestone 6 — portfolio/research finish

- Publish a versioned release and short technical report.
- Add architecture diagrams generated in CI.
- Add three measured plots to the visitor README.
- Provide a two-minute narrated demo and a replay-only quick start.
- Publish a small consented sample dataset with checksums and data card.
- If ML is pursued, train/evaluate one baseline and report failures, not only a
  successful clip.

## Recommended immediate sequence

Do M0-1 through M0-4, then M1-1, M1-3, M1-5, M1-7 and M1-6. Establish CI with
M2-1/M2-2 before the larger command-lease refactor. Implement recording M4-1
through M4-4 before base driving. This order improves safety and portfolio value
at every intermediate commit rather than creating one long feature branch.
