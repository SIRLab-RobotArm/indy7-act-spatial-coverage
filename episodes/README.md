# Raw HDF5 episodes

The original recordings straight off the robot. This directory is excluded from
git because of its size - only this README is tracked. These files are the
reference material behind every training dataset and every published number. See
[`../docs/data_and_weights.md`](../docs/data_and_weights.md) for how to obtain
them.

## Directories

| Directory | Contents | Used for the main experiment |
| --- | --- | --- |
| `main_20260817/` | The final 220 RGB-only successful demonstrations: 20 at each of P1-P9, plus 40 more at P5 | Yes |
| `pre_recollection_episodes/` | Recordings from before the recollection, kept for comparison and diagnosis | No |
| `development_episodes/` | Small episodes and command fixtures used while building the recorder and converter | No |
| `diagnostic_episodes/` | Records of board-shift, exposure, timestamp-jitter and teleoperation investigations | No |

`main_20260817/README.md` also contains progress snapshots taken during
collection. For the final count, this file and
[`../docs/experiment_design.md`](../docs/experiment_design.md) are authoritative:
220 episodes.

## Structure of the final data

```text
main_20260817/
├── p1/ ... p9/       raw HDF5, one directory per position
└── README.md         notes taken during collection
```

Each file contains the fixed overhead D435 colour stream, end-effector pose,
six-axis joint state, gripper state and timestamps. Depth and raw actions are not
part of these recordings. Converted datasets and the per-condition training
subsets are in [`../datasets/`](../datasets/README.md).

## Handling rules

- Never hand-edit HDF5 contents, attributes, filenames or UUIDs.
- If you find a file that fails QA or is incomplete, do not delete it. Move it
  somewhere diagnostic and write down why.
