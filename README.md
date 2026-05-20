# vive_teleop

## Services in docker-compose:

## bridge

### What is it?

It is ros1 and ros2 bridge.
https://github.com/ros2/ros1_bridge
Noetic -> Humble is not officially supported, so I stick with Noetic -> Foxy

### Why is it needed?
To isolate ROS1 environment on the robot and expose it to other services as ROS2.
It allows to use more modern tech stack, such as ROS2 Humble instead of ROS1.
Bridge itself uses Foxy, so communication between it and more modern services might be hard.
I recommend sticking to Humble (it is not eol yet!) or Foxy.

## ros2_app

### What is it?

It is WebRTC server that will be used to communicate between peripheral devices such as vr, joysticks, cameras, etc.
It runs on ROS2 Humble.

### Why is it needed?
To isolate subscriber/publisher logic from peripheral devices and allow easier changes in future.
It will have API with explanation on how to consume such endpoints in future.
For now it exposes WebRTC signaling on 0.0.0.0:8088:

- `/offer` answers receive-only video peers with the raw camera image from `/xtion/rgb/image_raw`.
- `/input_offer` answers WebRTC data-channel peers and forwards received messages to the mock ROS2 publisher on `/vive/input_mock`.

## unity-vr-headset

### What is it?

It is a unity project which handles head teleoperation.

### Why is it needed?

To render whatever robot sees to vr, but for now it is only for debugging, since bridge implementation throttles camera output down to ~23FPS.
Unless the robot gets updated to ROS2 it is unusable due to fatigue and dizziness. Recommended VR FPS is 72-120, with 90 being a sweet spot.
It is necessary to present a good MVP. 

### Whats the input and output?
It should consume WebRTC and serve the image to both lenses.
It should output the headset position using a similiar WebRTC server (WIP, for now just return some stream of data)
In future it should output joystick input as well.

## coturn

### What is it?

A TURN server which allows connection from other services to the WebRTC server

### Why is it needed?

Because WebRTC needs it to function properly with current network setup.

## How to run this project?

1. sudo docker compose up --build
2. python3 -m http.server 8000
3. Launch unity and open unity-vr-headset as a project
4. Launch steamvr
5. Press play
