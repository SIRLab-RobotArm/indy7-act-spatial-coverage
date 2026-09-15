# Driving the hardware by hand

Four shell aliases start the Indy7 driver, the Mark7 hand, the gripper preset
services and the Xbox teleoperation node, each in its own terminal. This is how
demonstrations were recorded and how the hardware is checked by hand.

> **Safety.** These commands move a six-axis industrial arm. Clear the space
> around the robot, keep the emergency stop within reach, and stay at the
> keyboard. The teleoperation alias below deliberately disables the software
> workspace limits, so holding a direction on the controller keeps moving the arm
> until you let go.

This document is for manual operation only. The completed experiment's 220
recordings and its 8-position evaluation procedure are described in
[`experiment_design.md`](experiment_design.md). Do not use these aliases to add
runs to the finished dataset or evaluation schedule.

| Alias | What it starts |
| --- | --- |
| `grip_indy` | The Indy7 ROS driver |
| `grip_mark7` | The Mark7 hardware interface and ros2_control |
| `grip_gripper` | The Mark7 open/grasp preset services |
| `grip_xbox` | The Xbox controller teleoperation node |

## 1. Install the aliases

Put them in `~/.bash_aliases`, which `~/.bashrc` loads automatically on Ubuntu.
An already-open terminal picks them up with `source ~/.bash_aliases`.

Replace `<REPO>` with the path to your clone, and fill in your own robot IP and
device paths - or better, set them once in `config/local.env` and let
`scripts/env.sh` supply them.

```bash
# Indy7 + Mark7 Xbox teleoperation
alias grip_indy='cd <REPO> && source scripts/env.sh && grip_source_ros && ros2 run indy_driver indy_driver.py --ros-args -p indy_ip:=${GRIP_INDY_IP} -p indy_type:=indy7 -p enforce_workspace:=false -p vel_ratio:=0.8 -p acc_ratio:=7.0'

alias grip_mark7='cd <REPO> && source scripts/env.sh && grip_source_ros && ros2 launch pipet_hand_mark7_driver mark7_hardware.launch.py port:=${GRIP_MARK7_PORT} use_mock_hardware:=false use_rviz:=false'

alias grip_gripper='cd <REPO> && source scripts/env.sh && grip_source_ros && ros2 run pipet_hand_mark7_teleop grip_preset_node --ros-args --params-file "${PROJECT_ROOT}/ros2_ws/src/mark7/pipet_hand_mark7_driver/config/grip_presets.yaml"'

alias grip_xbox='cd <REPO> && source scripts/env.sh && grip_source_ros && ros2 run indy7_teleop xbox_servo_node --ros-args -p indy_ip:=${GRIP_INDY_IP} -p input_backend:=linuxevdev -p event_device:=${GRIP_XBOX_DEVICE} -p linear_step_mm:=0.5 -p debug_input:=true'
```

### Finding your device paths

```bash
ls -l /dev/serial/by-id/                          # the Mark7's Arduino Micro
ls /dev/input/by-id/ | grep -i event-joystick     # the Xbox controller
rs-enumerate-devices -s                           # the RealSense serial number
```

Always prefer the `by-id` paths over `/dev/ttyACM0` and `/dev/input/js0`: they
survive reconnection and reboots, which the numbered ones do not.

## 2. Start order

Open four terminals and run them in this order. Each process stays in the
foreground of its own terminal.

**Terminal 1 - Indy7:** `grip_indy`, then check for

```text
Indy connector has been initialised.
ROBOT IP: <your robot ip>
ROBOT TYPE: indy7
```

**Terminal 2 - Mark7:** `grip_mark7`. Confirm the serial port opened and the
controller activated.

**Terminal 3 - gripper services:** `grip_gripper`, then wait for

```text
Grip preset node ready
```

**Terminal 4 - Xbox:** `grip_xbox`, then wait for `indy_srv connected` before
touching the controller.

## 3. Settings that matter

### ROS discovery

Every alias sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (via
`scripts/env.sh`). On a machine with several network interfaces, ROS nodes on the
same PC can otherwise fail to discover each other. If even one of the four runs
with a different discovery range, services and topics will go missing.

### Indy7

- Workspace boundary check: `enforce_workspace=false`
- Velocity ratio `0.8`, acceleration ratio `7.0`
- Rotation commands are still blocked, and the communication watchdog still runs

Because the workspace boundary is off, motion accumulates for as long as you hold
a direction. This alias is for slow checks before the workspace has been
measured. It is **not** the configuration used for data collection - that uses
the launch files, with measured `workspace_min_mm` / `workspace_max_mm` and
`workspace_configured=true`.

### Xbox

The default motion step is 0.5 mm per press, and `debug_input=true` prints the
raw button state, which is useful when a pad reports buttons differently.

## 4. Checking and stopping

```bash
type grip_indy grip_mark7 grip_gripper grip_xbox    # are they defined?
ps aux | grep -E 'indy_driver.py|ros2_control_node|grip_preset_node|xbox_servo_node' | grep -v grep
```

Stop each process with `Ctrl+C` in its own terminal. Never run two copies of the
same node - they will fight over the camera, `/indy_srv` and the ZMQ endpoint.

## 5. Controller mapping

| Input | Action |
| --- | --- |
| D-pad | Continuous X/Y motion |
| LT / RT | Z down / up |
| A | Close gripper |
| B | Open gripper |
| BACK | Stop teleoperation |
| BACK + A | Recover from an Indy fault |
| BACK + B | Move to the zero pose |
| BACK + Y | Move to home |
| BACK + LB / RB | Decrease / increase the motion step |
| MODE | Enter or leave end-effector teach-in |

## 6. End-effector teach-in

The MODE teach-in feature exists for implementation checks and separate
calibration work. **It was not used for the Q1-Q4 evaluation positions** - those
were kept as positions where the policy had never been shown a grasp. Use the
procedure below only for calibration work.

Press MODE to enter, then for each sample:

```text
B open -> BACK+Y home -> stand the pipe up again -> approach -> A close
       -> check visually -> X save
```

Repeat that cycle from home three times at the same position. After the first
sample, a save is refused until you reopen the gripper.

| Teach-in input | Action |
| --- | --- |
| `X` | Save the current end-effector pose and joint angles |
| `Y` | Discard the last sample at this position |
| `LB` / `RB` | Previous / next position |
| `A` / `B` | Close / open gripper |
| `BACK+Y` | Move to home |
| `MODE` | Leave teach-in |
| `START` | Blocked |

If the MODE button does not register, check button 8 in the `debug_input=true`
output. Some pads do not report MODE at all; on those, start the node with
`teach_toggle_button:=10` to use the right stick click instead.

After restarting the Indy driver, restart the Xbox node too. If the two disagree
about teleoperation state you will see repeated warnings of the form
`Ignoring /indy/teleop_pose because task-relative teleop is not active`.

## 7. The evaluation coordinates were not taught by hand

The target coordinates and distances used in the finished evaluation come from
automatic computation over the recorded demonstrations, not from manually saved
teach-in values:

```bash
cd /path/to/indy7-act-spatial-coverage
source scripts/env.sh && grip_activate_conda
python scripts/eval/prepare_targets.py
```

The output must report `evaluation_distances_ready: true`.
