# Evaluation protocol amendment — 2026-08-25

This is a retrospective amendment made after main Trials 1--3 had been
completed and their outcomes were known. It must not be described as a
pre-registration. The three completed rollouts remain valid and are not
deleted or repeated.

## Rollout count

The planned repeat count is increased from two to five for every one of the
96 model-position cases:

```text
4 conditions × 3 training seeds × 8 positions × 5 repeats = 480 rollouts
```

The original 192-row schedule is preserved byte-for-byte as rows 1--192 of the
new schedule. Repeats 3--5 are appended as rows 193--480. Each 24-trial
session remains balanced: every model appears twice, every condition six
times, every seed eight times, and every position three times.

Increasing only poorly performing cases is prohibited because that would make
the sample size depend on observed policy outcomes. Any increase beyond five
must be declared as another amendment and add the same complete repeat block
to all 96 cases. Hardware-fault and externally moved-object attempts continue
to follow the separately frozen replacement rule.

## Grasp target and endpoint errors

For G5/G1/G6/G8, the target is the mean EEF XYZ at the first gripper 0->1
transition in all accepted demonstrations collected at the corresponding
P5/P1/P6/P8 location. The same target is used for every condition and seed.

Q1--Q4 already had targets frozen before the first main rollout: the
equal-weight mean of the four surrounding grid targets. Those targets remain
the primary endpoint-error references because replacing them after observing
three rollout outcomes would move the scoring target post hoc.

Five manually executed reference grasps may be collected at each Q location
as a separately identified, reference-only calibration set. They are never
added to a training dataset, never used to tune or select a policy, and never
passed to the policy executor. Their mean and spread are reported as a
sensitivity check against the pre-existing interpolated Q target; they do not
silently replace the primary target.

Per rollout, analysis records:

- target XYZ;
- EEF XYZ interpolated at the first policy gripper-close event;
- `E1`: XY error at close;
- `E2`: signed Z error at close;
- `close_error_3d_mm`: 3-D error at close;
- `E3`: minimum pre-close XY distance, which remains available when no close
  occurs;
- operator-scored success/failure and the frozen failure code.

Operator visual judgment of actual PVC grasp remains authoritative. Endpoint
error supplements failure analysis; it does not turn a missed grasp into a
success.

## Result artifacts

Raw evidence remains the source of truth:

- `trace.parquet`: time-series robot/policy values;
- `events.json`: start, close, stop, result and contamination events;
- `meta.json`: condition, seed, position, repeat and target metadata;
- `rollout.bag/`: original ROS messages.

Final analysis writes both a JSON summary and a flat per-rollout CSV table.
JSON preserves nested metadata and model results; CSV is the paper-facing
table used for checking, plotting, statistics and supplementary release.

## Schedule identity

- schedule seed: `20260824`
- old 192-row SHA-256:
  `77acadde86eb90ec490ccca87435cc68e034e4b24ce255fd15259d451bba8e89`
- amended 480-row SHA-256:
  `f6e26ce805d0f4dc0d308fb99f1a179978ee544796603dd35e12575abd0a193b`
