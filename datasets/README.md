# ACT training datasets

This directory holds the RGB-only LeRobot v3 datasets for the
`main_recollection_20260817` experiment. It is excluded from git because of its
size - only this README is tracked. See
[`../docs/data_and_weights.md`](../docs/data_and_weights.md) for how to obtain
the data.

## The data contract, shared by all five datasets

- Source: 220 successful demonstrations recorded with an Indy7 and a Mark7 hand
- Camera: fixed overhead RealSense D435, colour only, depth unused
- Raw observation rate: 20 Hz
- Conversion: each raw demonstration becomes four 5 Hz trajectories, at phase
  offsets 0, 1, 2 and 3
- Image: `observation.images.overhead`, stored as `(240, 320, 3)`, fed to the
  model as `(3, 240, 320)`
- State, 10 values: end-effector XYZ (3) + joint angles (6) + gripper state (1)
- Action, 4 values: end-effector XYZ delta to `t+4` (3) + gripper command (1)

## The five datasets

| Directory | Raw demos | LeRobot trajectories | Frames | Role |
| --- | ---: | ---: | ---: | --- |
| `main_recollection_20260817_rgb_all` | 220 | 880 | 176,837 | The full conversion. The source that A-D are regenerated from |
| `main_recollection_20260817_rgb_a` | 60 | 240 | 45,614 | Condition A: 60 demos, all at G5 |
| `main_recollection_20260817_rgb_b` | 60 | 240 | 48,021 | Condition B: 20 each at G1, G6, G8 |
| `main_recollection_20260817_rgb_c` | 60 | 240 | 49,136 | Condition C: 6-7 each across G1-G9 |
| `main_recollection_20260817_rgb_d` | 180 | 720 | 146,332 | Condition D: 20 each at G1-G9 |

`rgb_a` through `rgb_d` are the direct training inputs for the 12 models
(4 conditions x 3 seeds). `rgb_all` was not trained on directly, but is kept
because every subset derives from it.

## What the conditions compare

- **A vs B vs C**: raw demonstration count fixed at 60, spatial spread varied
- **C vs D**: the 9-position spread held fixed, count varied between 60 and 180
- Each condition trains seeds 0, 1 and 2 from the same frozen membership
- Every run uses the final checkpoint at 100,000 optimisation steps

## Reference metadata

Each dataset's schema and counts are in its own `meta/info.json`, and its
statistics in `meta/stats.json`. Condition membership and the regeneration record
are authoritative in:

```text
../experiment/manifests/main_recollection_20260817_rgb_all.json
../experiment/manifests/main_recollection_20260817_condition_{a,b,c,d}_uuids.json
../experiment/manifests/main_recollection_20260817_condition_{a,b,c,d}_dataset.json
```

## Handling rules

- Do not hand-edit directory names, episode order, parquet or image content, or
  metadata.
- When rebuilding a subset, use the frozen UUID manifest. Do not re-select
  episodes by position rule - that would silently produce a different dataset.
- Never mix new data into an existing dataset. Use a new experiment ID and a new
  output directory.
