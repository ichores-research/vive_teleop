# Reference Sources

Primary/upstream sources used for this audit:

- [MoveIt Servo Humble tutorial](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
  — standard Servo command path, singularity and collision-handling intent.
- [ros2/teleop_twist_joy](https://github.com/ros2/teleop_twist_joy/tree/humble)
  — enable-button-first joystick mapping and standard Twist boundary.
- [Nav2 velocity smoother](https://github.com/ros-navigation/navigation2/tree/humble/nav2_velocity_smoother)
  — velocity/acceleration/deadband smoothing and timeout concepts.
- [Nav2 collision monitor tutorial](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)
  — downstream collision monitoring for base velocity commands.
- [ros2/rosbag2](https://github.com/ros2/rosbag2)
  — recording/playback, explicit topics, QoS overrides, MCAP, services,
  snapshots and lost-message statistics.
- [ROS 2 Quality Guide](https://docs.ros.org/en/ros2_documentation/jazzy/The-ROS2-Project/Contributing/Quality-Guide.html)
  — ament-integrated static analysis and test dependencies.
- [ROS 2 Humble QoS concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
  — keep-last, reliability, durability and sensor-data tradeoffs.
- [ROS 2 Humble security tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Introducing-ros2-security.html)
  and [deployment guidance](https://docs.ros.org/en/ros2_documentation/humble/Tutorials/Advanced/Security/Deployment-Guidelines.html)
  — DDS authentication, encryption, access controls and enclaves.
- [Unity-Technologies/com.unity.webrtc](https://github.com/Unity-Technologies/com.unity.webrtc)
  — official package structure, samples, tests, license and supported Unity path.
- [Official WebRTC samples](https://webrtc.github.io/samples/)
  — peer/signaling lifecycle examples.
- [Docker build best practices](https://docs.docker.com/build/building/best-practices/)
  — multi-stage builds, minimal runtime dependencies, pinning and non-root users.
- [GitHub Unity `.gitignore`](https://github.com/github/gitignore/blob/main/Unity.gitignore)
  — generated Unity/crash/build patterns.
- [GitHub Actions Python testing guide](https://docs.github.com/en/actions/tutorials/build-and-test-code/python)
  — pytest, coverage and lint workflow patterns.
- [PAL Robotics repositories](https://github.com/pal-robotics)
  — upstream TIAGo, PMB2, simulation and controller package boundaries.

Peer-project comparisons:

- [Quest2ROS2](https://github.com/Taokt/Quest2ROS2) — ROS 2/VR teleoperation,
  dedicated bringup/custom messages and research presentation.
- [SpesRobotics/teleop](https://github.com/SpesRobotics/teleop) — WebXR
  teleoperation, reusable robot interfaces, simulation examples and tests.
- [SO-101 ROS Physical AI](https://github.com/legalaspro/so101-ros-physical-ai)
  — end-to-end teleoperation, rosbag episodes, visualization, conversion and
  policy workflow.
- [OpenArm](https://github.com/enactic/openarm) — separated dataset/simulation
  ecosystem and open-source project governance.

Reference projects are not treated as infallible. Their relevant patterns were
compared with this repository's constraints; recommendations still require
validation on the installed ROS Humble/TIAGo/Unity versions.
