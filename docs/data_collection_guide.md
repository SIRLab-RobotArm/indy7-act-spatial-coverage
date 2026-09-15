# RGB-only demonstration collection guide

Data collection for this experiment is finished. This document preserves the data
contract every recorded episode satisfies, and the procedure to follow if you
deliberately collect more. **Do not mix new episodes into the existing dataset.**

## 1. What was collected

Location: `episodes/main_20260817`

| Position | Usable episodes |
| --- | ---: |
| G1, G2, G3, G4 | 20 each |
| G5 | 60 |
| G6, G7, G8, G9 | 20 each |
| Total | 220 |

Recording ran sequentially, one position at a time, in blocks. It was not
randomly interleaved; that deviation is recorded in
`experiment/preregistration.md`.

## 2. Fixed experimental conditions

- Object: 25 cm rigid PVC-U pipe, non-pressure grade, straight
- Outer diameter 2.5 cm, inner diameter 2.0 cm
- Specification: 20x4M, KS M 3404, SDR 9 / VN straight pipe
- No overgrip or added surface texture on the object
- Camera: fixed overhead RealSense D435, colour, 640x480
- Depth is never recorded, trained on, or evaluated
- Table and camera are fixed; a physical barrier bounds the reachable workspace
- Lift success is judged visually by the operator

## 3. The episode contract

One episode is a single successful demonstration: start at home, approach, grasp,
lift.

- Observations: colour image, end-effector pose, joint state, gripper state
- Action: end-effector motion to `t+4` in the future, plus the gripper command at
  that same target time
- Raw recording rate: about 20 Hz
- Training conversion: four phase-offset 5 Hz trajectories per episode
- Maximum allowed timestamp skew between image and state: 30 ms
- The gripper closes exactly once and does not reopen
- Every episode in the main dataset carries a success label

The raw HDF5 files are the reference material for training and for reproducing
the paper. Never hand-edit them, and never identify an episode by filename - use
its UUID and the frozen manifest.

## 4. If you need to collect new data

Create a new experiment ID and an empty output directory first. Do not write into
`episodes/main_20260817`.

```bash
cd /path/to/indy7-act-spatial-coverage
export GRIP_OUTPUT_DIR="$PWD/episodes/<new_experiment>/p1"
export GRIP_DATA_BLOCK=main
export GRIP_SESSION_ID=<new_session>
export GRIP_OPERATOR_ID=<operator>
./scripts/collection/collect_p1.sh
```

For P2 through P9, change both the script (`collect_p2.sh` ... `collect_p9.sh`)
and the `pN` component of `GRIP_OUTPUT_DIR`. Close RealSense Viewer and any other
launch file that claims the camera first - exactly one collection graph may run
at a time.

The arm, hand and controller drivers are started separately; see
[`manual_teleop.md`](manual_teleop.md). If the controller's event device changes,
re-check the current symlink under `/dev/input/by-id/`.

## 5. Recording one episode

1. Move to home, open the gripper, confirm the PVC pipe's position
2. Confirm the recorder is idle
3. Press START to begin recording
4. Perform approach, grasp and lift once, in one natural motion
5. Stop recording and save with the success label
6. **Wait for the final HDF5 path to appear in the terminal**
7. Return to home and open the gripper before the next episode

Do not press START again before the file path appears. Restarting mid-save
produces timestamp gaps or truncated episodes. If the arm controller reports an
overrun, the controller disconnects, or the camera readback errors, that file
does not go into the main dataset.

## 6. Quality assurance before a batch counts

Freeze a new batch into its own manifest only after checking all of:

- The HDF5 file opens and its schema version is correct
- Colour frames are 480x640x3, read back correctly, and are not a frozen image
- Timestamps increase monotonically, within the allowed jitter
- Image and state streams are synchronised
- No NaN or Inf values
- Exactly one gripper close transition
- Sufficient lift after that close
- The first-close end-effector positions cluster, and the contact is visually
  correct

Do not delete a file that fails QA. Move it to a diagnostic directory and record
the reason in that directory's README.

Merging new data into the existing A-D conditions is not possible. Treat it as a
new experiment and freeze new manifests, datasets, models and an evaluation
schedule.

## 7. Conversion and training

The completed conversion and training conditions are described in
[`experiment_design.md`](experiment_design.md). The conversion is implemented in
`ai/convert/build_dataset.py`, condition membership in
`ai/convert/freeze_conditions.py`, and training is entered through
`ai/train/run_train.py` and `scripts/train/run_main_matrix.sh`. The final 480
trials are summarised in
[the results README](../experiment/evaluation/main_recollection_20260817/results/README.md).
