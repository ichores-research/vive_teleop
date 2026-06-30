# moveit_server Context

## Responsibility

`moveit_server` owns robot command generation. It converts typed `/vive/*` topics into direct head/gripper trajectories and MoveIt Servo wrist motion.

## Key Files

- `moveit_server/src/vive_moveit_server/vive_moveit_server/vive_moveit_server.py`: main ROS 2 node, parameters, head control, gripper control, timers.
- `moveit_server/src/vive_moveit_server/vive_moveit_server/teleop_data.py`: subscriptions for teleop input.
- `moveit_server/src/vive_moveit_server/vive_moveit_server/arm_movement.py`: deadman clutch, TF wrist anchor, workspace constraints, Servo pose publication.
- `moveit_server/src/vive_moveit_server/vive_moveit_server/servo_pose_bridge.py`: absolute pose target -> Cartesian twist command.
- `moveit_server/src/vive_moveit_server/launch/vive_moveit_server.launch.py`: top-level launch.
- `moveit_server/src/vive_moveit_server/launch/servo_runtime.launch.py`: MoveIt Servo and pose bridge launch.
- `moveit_server/src/vive_moveit_server/config/tiago_single_params.yaml`: teleop parameters.
- `moveit_server/src/vive_moveit_server/config/tiago_servo.yaml`: MoveIt Servo parameters.
- `moveit_server/src/vive_moveit_server/config/servo_pose_bridge.yaml`: pose bridge gains, timeouts, and velocity limits.

## ROS 2 Inputs

- `/vive/head_pose`
- `/vive/hand_target_pose`
- `/vive/hand_target_active`
- `/vive/gripper_opening`
- `/joint_states`
- TF for current wrist pose and robot model.

## ROS 2 Outputs

- `/head_controller/joint_trajectory`
- `/gripper_controller/joint_trajectory`
- `/servo_node/pose_target_cmds`
- `/servo_node/pose_target_active`
- `/servo_node/delta_twist_cmds`
- `/arm_controller/joint_trajectory` is produced by MoveIt Servo, not directly by `vive_moveit_server`.

## Control Model

- Head: HMD yaw/pitch -> pan/tilt joints at fixed rate with deadband and limit scaling.
- Wrist: deadman press anchors current controller pose to current robot wrist TF; only relative controller motion after the press is applied.
- Servo bridge: computes pose error plus feed-forward target velocity, publishes physical `TwistStamped` speed units.
- Gripper: normalized opening -> two finger joint positions with deadband and velocity-aware duration.

## Data Stability Notes

- `hand_target_timeout_sec` and `target_timeout_sec` protect the wrist path from stale input.
- Deadman release clears target state and publishes repeated halt commands.
- Workspace constraints clamp wrist target distance and z range before Servo.
- Head and gripper paths should be kept symmetric with the arm path by adding stale-input and stale-joint-state checks where needed.
- The default Servo profile is tuned for this robot: bridge velocity caps are
  disabled and Servo collision checking is intentionally off because its
  behavior is too aggressive for this deployment. Do not make recorder or QA
  work conditional on enabling it.

## Future Dataset Recording

The proposed recorder uses `/servo_node/pose_target_active` as the effective
arm-action gate and records pose target, generated twist, controller trajectory,
and measured state as distinct streams. Do not collapse these stages into one
"action" field; the future policy interface determines which stage becomes the
training label.

See `docs/data-recording/dataset-contract.md` and
`.agents/data-recorder-context.md`.

## Update Checklist

When changing motion behavior:

- Update the relevant config YAML.
- Update `scripts/check-teleop-runtime.sh` if startup validation should enforce the new invariant.
- Update `README.md`, `docs/architecture/component/moveit-server.puml`, `docs/architecture/component/ros-topic-boundary.puml`, `docs/architecture/communication/data-flow.puml`, and the relevant class diagrams.
- Add or extend tests in `moveit_server/src/vive_moveit_server/test/`.
