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
For now it should expose WebRTC server with raw camera image on 0.0.0.0:8088/offer

## steamvr

### will do later

## coturn

### What is it?

A TURN server which allows connection from other services to the WebRTC server

### Why is it needed?

Because WebRTC needs it to function properly with current network setup.