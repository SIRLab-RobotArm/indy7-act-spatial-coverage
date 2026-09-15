# Experiment design: RGB-only ACT data collection and evaluation

This is the single reference for where the data lives, how many episodes there
are, what the A-D training conditions mean, and how the evaluation positions were
chosen. Where this document and the frozen artifacts disagree, the artifacts -
`experiment/positions.yaml`, the membership manifests, and `schedule_full.csv` -
are authoritative.

> Collection, conversion, training of all 12 models and all 480 evaluation
> trials are complete. Final numbers are in
> [the results summary](../experiment/evaluation/main_recollection_20260817/results/README.md).

## 1. Scope

- The only sensor input is the colour image from a fixed overhead RealSense D435.
- Depth is not recorded, not aligned, not used for training or evaluation. There
  is no RGB-D arm of this experiment.
- The object, camera, lighting and gripper are held fixed. The only thing that
  varies between conditions is the **number and XY placement** of the
  demonstrations.
- We do not claim broad extrapolation. The primary measurement is whether a
  policy reproduces grasps at positions it was trained on; the secondary
  measurement is interpolation to nearby positions inside the same grid.

## 2. Position layout

A 3x3 collection grid sits inside the safe workspace, centred on `G5`. The
physical markers were placed by hand, roughly 15 cm apart horizontally and 13 cm
vertically. The precise end-effector reference used for analysis is not the
marker but the mean pose at first gripper close across the successful
demonstrations, recorded in `experiment/positions.yaml`.

```text
G1 ----- G2 ----- G3
 |   Q1   |   Q2   |
G4 ----- G5 ----- G6
 |   Q3   |   Q4   |
G7 ----- G8 ----- G9
```

Writing the centre as `(x0, y0)` in the Indy base frame:

| Position | XY |
| --- | --- |
| G1 | `(x0-dx, y0+dy)` |
| G2 | `(x0, y0+dy)` |
| G3 | `(x0+dx, y0+dy)` |
| G4 | `(x0-dx, y0)` |
| G5 | `(x0, y0)` |
| G6 | `(x0+dx, y0)` |
| G7 | `(x0-dx, y0-dy)` |
| G8 | `(x0, y0-dy)` |
| G9 | `(x0+dx, y0-dy)` |

`Q1` to `Q4` are the arithmetic centres of their four neighbouring grid
positions. All four lie inside the convex hull of the 3x3 grid, so they are
nearby interpolation targets rather than extrapolation.

## 3. The collected data: 220 episodes

The count is **usable demonstrations** - episodes that passed inspection and the
exclusion rules. Failed, discarded and corrupted recordings are not part of the
220, and their reasons are recorded separately.

| Position | Episodes |
| --- | ---: |
| G1 | 20 |
| G2 | 20 |
| G3 | 20 |
| G4 | 20 |
| G5 | 60 |
| G6 | 20 |
| G7 | 20 |
| G8 | 20 |
| G9 | 20 |
| **Total** | **220** |

Collection ran position by position, in blocks, in the order G1, G2, ..., G9,
with the extra G5 episodes gathered as another block. The randomised, balanced
interleaving that was originally planned did **not** happen. Time of day,
operator fatigue and lighting drift are therefore confounded with position. This
is disclosed as a limitation in the paper and in
`experiment/preregistration.md`; the actual procedure is not described after the
fact as randomised.

## 4. The A-D training conditions

Four subsets are drawn from the 220 originals. The selection rule and the exact
list of episode UUIDs were frozen in an immutable manifest **before any model
results were seen**. Seeds 0, 1 and 2 share the same condition manifest, so what
differs between the three models of a condition is training randomness, not data.

| Condition | Episodes per position | Total | What it isolates |
| --- | --- | ---: | --- |
| A | 60 at G5 | 60 | Everything concentrated at one position |
| B | 20 each at G1, G6, G8 | 60 | Same count, spread over 3 positions |
| C | Distributed over all nine (below) | 60 | Same count, spread over 9 positions |
| D | 20 each at G1-G9 | 180 | Same 9-position spread, three times the data |

Condition C's exact allocation:

```text
G1=7   G2=6   G3=7
G4=7   G5=6   G6=7
G7=7   G8=6   G9=7
```

The two comparisons that matter: `A vs B vs C` holds the episode count at 60 and
varies spatial spread; `C vs D` holds the 9-position spread and varies the
episode count. Every condition is RGB-only, and every condition is trained with
three ACT seeds, giving 12 models in total.

Every training run uses exactly **100,000 optimisation steps**, regardless of
condition or seed. Checkpoints are written every 20,000 steps, leaving `20k`,
`40k`, `60k`, `80k` and `100k`. Interrupted runs are restarted rather than
resumed, and evaluation uses only each run's final `100k` checkpoint.

## 5. Evaluation positions and trial count

All 12 models are evaluated at the same 8 positions.

| Evaluation ID | Physical position | Kind |
| --- | --- | --- |
| `eval_1` | G5 | Same as a collection grid point |
| `eval_2` | G1 | Same as a collection grid point |
| `eval_3` | G6 | Same as a collection grid point |
| `eval_4` | G8 | Same as a collection grid point |
| `eval_5` | Q1 | Interpolated, inside the grid |
| `eval_6` | Q2 | Interpolated, inside the grid |
| `eval_7` | Q3 | Interpolated, inside the grid |
| `eval_8` | Q4 | Interpolated, inside the grid |

`eval_1` to `eval_4` copy the mean successful-demonstration pose of their grid
position verbatim, so one physical location never ends up with two different
target coordinates. The `Q1`-`Q4` references are fixed automatically before
evaluation as the equally weighted mean of their four surrounding grid means.
**No demonstrations and no manual teach-in were performed at the Q positions.**
A separate Q reference grasp is permitted only as a sensitivity analysis, never
as a retroactive replacement for the primary reference.

- 5 rollouts per position
- 8 x 5 = 40 rollouts per model
- 4 conditions x 3 seeds = 12 models
- **12 x 40 = 480 rollouts**

The repeat count was raised from 2 to 5 on 2026-08-25, after trials 1-3 of the
main evaluation had already run. The existing 192 rows are preserved as an exact
prefix of the new schedule. The change is documented in
`experiment/evaluation/main_recollection_20260817/protocol_amendment_20260825.md`.

The primary result is the success rate and failure pattern at the four grid
positions. The `Q1`-`Q4` results are reported separately as the secondary,
interpolation result. For every evaluation point we also record `d`, the distance
to the nearest training position for that condition.

Trials disturbed by `object_move` or `hardware_fault` are reported as having
occurred, excluded from the success-rate denominator, and repeated under a new
attempt ID for the same schedule row. Other policy failures are never repeated.
The inferential analysis uses `condition * d` with the repeated structure of
`model_code` and position; sessions were balanced in the schedule and checked
with a time-trend sensitivity analysis. The pooled rollout Wilson interval is
**not** to be read as a confirmatory interval standing in for independent model
replicates - the mean and standard deviation across the three models of each
condition are reported alongside it.

## 6. Pilot positions and physical markers

`pilot_1` and `pilot_2` exist only for operator practice, pipeline checks and
tuning the success criterion. They are excluded from all training and evaluation
data. The files list 19 position IDs, but the number of distinct physical
locations is `9 grid + 4 Q + 2 pilot = 15`.

Markers are small, identical in size and colour, and hidden under the base of the
PVC pipe, so that no position carries a distinguishing visual cue that a policy
could exploit.
