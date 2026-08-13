# Soldering Robot Dashboard

PyQt5 operations screen for the six-motor ROS stack, perception pipeline, and
ConvNeXt W&B runs.

## Safety contract

- Starting the dashboard starts no bridge, motor node, camera, or training job.
- Motor state comes from `/motors/motor_N/telemetry`; the UI does not infer a
  healthy motor merely from a node or topic name.
- The ARM badge follows `/soldering/software_armed`, which is published by the
  fail-closed hardware setup node after fresh, valid feedback checks.
- PCM commands are locked by default. ARM and slot playback also require a
  confirmation dialog. A submitted command is not shown as successful until
  the PCM daemon response and subsequent status arrive.
- The PCM STOP button is explicitly not an emergency stop. Use the physical
  E-Stop for an emergency.
- `/phorce/feedback` uses `qos_profile_sensor_data` and its high-rate callback
  records arrival time only.

## Run

```bash
source /home/phorce/hackathon/.venv/bin/activate
source /opt/ros/humble/setup.bash
source /home/phorce/hackathon/soldering_robot_ws/install/setup.bash
ros2 run soldering_dashboard dashboard
```

The hardware, simulated motor, vision, or PCM daemon stacks must be launched
separately. This avoids a monitoring action unexpectedly starting hardware.

W&B defaults to entity `donghyeok8649-kaist` and project
`soldering-vision`. Both fields are editable. Authentication is read from the
normal W&B configuration (`~/.netrc`); the API key is never displayed or copied
into the UI.

## Tabs

- **Motor / control**: six-axis table, selected-axis time series, actual setup
  arm state, and locked PCM service commands.
- **ROS topics**: publisher/subscriber counts plus measured receive rate and
  age. A publisher without fresh messages is not marked active.
- **Vision**: annotated camera topic and `GeometryObservation` fields.
- **Training / W&B**: newest run state, loss, validation accuracy, config,
  system-metric availability, and a link to the W&B run.
- **Logs**: filtered `/rosout` and local dashboard command responses.

## W&B fields currently produced by the trainer

`train_convnext.py` logs `epoch`, `train/loss`, `validation/accuracy`, and
`learning_rate`, plus the run config and best-model artifact. The dashboard is
already tolerant of the future `validation/loss` field.

