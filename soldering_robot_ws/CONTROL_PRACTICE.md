# Two-axis control practice

This exercise is for the two currently installed motors before calibration.

- PhACT port 2 is ROS `axis[0]`.
- PhACT port 9 is ROS `axis[7]`.
- The final six-axis mapping will be configured later; do not copy the draft
  `0..5` mapping from the planning document as a measured mapping.
- The exercise uses a preloaded PCM P-Vector motion slot. It does not stream raw
  joint positions and never repeats a motion automatically.

## Build

```bash
cd /home/phorce/hackathon/soldering_robot_ws
source /home/phorce/hackathon/.venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```

## Simulation exercise

Terminal 1:

```bash
ROS_DOMAIN_ID=73 ros2 launch agx_bringup motion.launch.py slot_mask:=2
```

Terminal 2:

```bash
ROS_DOMAIN_ID=73 ros2 run soldering_control two_axis_practice \
  --target sim:demo --motion-id 1 --execute
```

The simulator checks the one-shot motion API. It does not simulate joint
feedback, geometry, or a P-Vector trajectory.

## Hardware observation only

With the approved local EtherCAT gateway running:

```bash
ROS_DOMAIN_ID=73 ros2 run soldering_control two_axis_practice --target robot
```

This prints position, velocity, current, voltage, temperature, and validity for
ports 2 and 9. It sends no motion.

## One real slot, once

Only after checking the robot area, physical E-Stop, PCM state, and the content
of the selected Studio/MotionMap slot:

```bash
ROS_DOMAIN_ID=73 ros2 run soldering_control two_axis_practice \
  --target robot --motion-id 1 --execute \
  --confirm-real MOVE-REAL-ROBOT-ONCE
```

The node blocks execution if feedback is missing, either axis is invalid or
faulted, the mechanism is already moving, or the slot is not in the PCM catalog.
After completion it reports the measured angular displacement of both motors.

Software cancellation is not an E-Stop. Use the physical E-Stop for an actual
emergency.

## Manual arm, then slots 4 and 5

The operator presses PCM function button 1 for about one second and waits for
the warning and boot-pose motion to finish. After confirming that the robot
area is clear and the physical E-Stop is reachable, run:

```bash
cd /home/phorce/hackathon/soldering_robot_ws
source /home/phorce/hackathon/.venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select soldering_control --symlink-install
source install/setup.bash

# Read-only preflight: slots 4 and 5 must both appear in the PCM catalog.
ros2 run soldering_control play_motion_4_5

# Real execution: slot 4 completes before slot 5 is submitted.
ros2 run soldering_control play_motion_4_5 \
  --execute --confirm-real PLAY-REAL-MOTIONS-4-5
```

This command never arms, stops, parks, or disables the servo. After both
motions complete, use PCM function button 2 manually. `Ctrl+C` and software
cancellation are not an E-Stop.
