# ICRiTA Indy7 RGB-only grip retrospective analysis plan

Status: **retrospective analysis plan, not preregistered**. The main-evaluation
protocol was first fixed on 2026-08-24 after collection and training. On
2026-08-25, after Trials 1--3 and their outcomes were known, the repeat count
was amended from two to five for all model-position cases. See
`evaluation/main_recollection_20260817/protocol_amendment_20260825.md`. Neither
version may be described as a pre-collection or pre-training registration.

## Confirmatory hypotheses and design

- H1: with 60 demonstrations fixed, broader XY coverage improves task success
  and reduces approach failures (`C > B > A`).
- H2: with all nine locations fixed, increasing demonstrations from 60 to 180
  improves success, with possible saturation (`D > C`).
- H3: distribution primarily changes approach/alignment failures, while purely
  mechanical slip failures may remain.
- Input is RGB-only. Depth collection, RGB-D training, and claims about depth
  are outside the experiment scope.
- Main collection contains 220 usable demonstrations: 20 at each of nine grid
  positions and 40 additional demonstrations at G5, for 60 total at G5.
- The intended balanced interleaving was not followed. Actual recollection was
  position-blocked P1, P2, ..., P9, with the additional G5 trials also collected
  as a block. This deviation is reported rather than hidden.
- Conditions: A=G5×60; B=(G1,G6,G8)×20; C=60 across all nine positions with
  rows `(7,6,7)`; D=all nine positions×20=180.
- Seeds 0, 1, 2 use the same immutable subset manifest within each condition.
- The exact geometry and full mapping are defined in
  `docs/experiment_design.md`.

## Frozen representation and training contract

- Observation rate 20 Hz. Four phase-split 5 Hz trajectories preserve every
  20 Hz observation while ACT chunks advance by 200 ms.
- Action: `[dx_mm, dy_mm, dz_mm, gripper]`, where XYZ is
  `ee_pose[t+4]-ee_pose[t]`; rotation is excluded and gripper is binary.
- State: EEF XYZ, six joint positions in radians, and binary gripper (10-D).
- RGB is area-resized to 240x320. No sensor depth is collected or used.
- Normalization: VISUAL IDENTITY, STATE MEAN_STD, ACTION QUANTILE10.
- ACT: ResNet18 ImageNet, CVAE enabled, chunk 40, execution 10, batch 64,
  AdamW wd 1e-4, gradient clip 10, peak LR 1e-4, 2k warmup, cosine to 1e-6,
  100k fixed steps, checkpoints every 20k steps, no resume. The 100k final
  checkpoint is used for the preregistered evaluation.

## Blind exclusions and gates

- Run QA once before condition injection. Any excluded raw episode is excluded
  from both visual conditions.
- Exclude corrupted/incomplete files, synchronization jitter above the frozen
  RGB/robot-state threshold of 15 ms,
  and safety-stop trials. Reasons are retained.
- G1, RGB camera G2, G5, and G6 must pass before main collection/training.
- G2 verifies RGB frame rate, fixed exposure/gain/white balance, PVC visibility,
  and robot occlusion at all 15 distinct physical locations.
- Main collection is invalid if camera geometry/settings, timestamp semantics,
  EEF meaning, gripper definition, task object, or success rule changes within
  the block.

## Outcomes and analysis

- All 12 models are evaluated at the same eight locations: G5, G1, G6, G8 and
  the four within-grid cell centers Q1~Q4, with five rollouts per location.
  This gives 40 rollouts/model and 480 total rollouts.
- Primary rollout outcomes are success and endpoint errors E1/E2/E3. Success
  requires the PVC to be grasped, EEF Z to rise at least 50 mm after the first
  close, and the PVC to remain held for at least 3 seconds. From smooth-controller
  Trial 4 onward, a fixed 60-second policy horizon is enforced. Reaching the
  horizon without success is a policy failure.
- Before the new 30 Hz main block began, the operator interface was simplified
  to a binary visual `success`/`failure` decision. Policy failures are stored as
  `operator_failure`; legacy detailed codes remain readable only for the
  excluded 5 Hz diagnostic records. Equipment faults or external object moves
  abort the attempt and are rerun under a new attempt ID.
- The operator records failure before physical contact when the policy is on a
  collision course. Collision-risk safety intervention, actual unintended
  contact, object toppling, closing away from the PVC, dropping it, failing the
  lift/hold rule, and policy timeout all count as `operator_failure` in the
  success denominator. Target distance is a continuous secondary outcome, not
  a binary failure threshold.
- Exact-grid `p_target` is the mean EEF XYZ at the first gripper 0->1 transition
  across all successful demonstrations at that grid. Each Q target is the
  equal-weight mean of its four surrounding grid targets. No demonstration or
  manual teach-in is performed at Q1~Q4 before evaluation. Distance `d` is
  planar XY Euclidean distance from an evaluation target to the nearest
  training grid target for that condition.
- Primary reporting uses the four exact grid evaluation positions. Q1~Q4 are
  a secondary local-interpolation analysis, not an extrapolation claim.
- Trials marked `object_move` or `hardware_fault` are reported but excluded
  from the policy-success denominator and replaced under the same scheduled
  model/position/repeat using a new attempt ID. Other policy failures are not
  excluded or repeated.
- Fit `condition * d + (1|model_code) + (1|position)` to success
  and each continuous error. The 12 condition-by-seed training runs are the
  model clusters; the three numeric seed labels are not treated as three
  rollout clusters shared across conditions. E1/E2 primary analysis is
  restricted to clean trials and E3 remains defined from the pre-stop
  trajectory even when no close occurs.
- Session is balanced by construction and is inspected as a temporal
  sensitivity analysis rather than added as a third random effect to this
  small primary model.
- Report pooled rollout-level Wilson intervals as descriptive intervals and
  also report mean and standard deviation of the three model success rates per
  condition. The limited model replication precludes strong confirmatory
  claims from small differences.
- Report all safety stops, E1/E2 missingness by condition,
  condition-blinded randomized rollout order, and Cohen's kappa on a random
  20% independently rescored video sample.
- The amended 480-trial order is
  `experiment/evaluation/main_recollection_20260817/schedule_full.csv`
  (SHA-256 `f6e26ce805d0f4dc0d308fb99f1a179978ee544796603dd35e12575abd0a193b`).
  Its first 192 rows exactly preserve the earlier schedule. Any increase beyond
  five repeats must add a complete repeat for all 96 cases rather than select
  cases based on observed success.
  It is split into twenty balanced 24-trial sessions.

## Retrospective execution amendment (2026-08-25)

After four scheduled rollouts, visible shaking was attributed to passing each
5 Hz cumulative ACT endpoint directly to Indy. The endpoint values, action
rate, chunks, gripper logic, and model are unchanged, but every 200 ms endpoint
is now delivered as six equal 30 Hz cumulative targets. Existing coarse-command
Trial 1--4 results are retained for diagnosis and excluded from the final
analysis; the 480-row schedule restarts at Trial 1 in
`experiment/rollouts/main_recollection_20260817_main_smooth30hz` after one
separate hardware pilot. This decision was made after the first four outcomes
were known and is not a prospective preregistration. Full details are in
`experiment/evaluation/main_recollection_20260817/protocol_amendment_20260825_smooth_execution.md`.

## Retrospective outcome amendment (2026-08-26)

After smooth-controller Trials 1--3 had been observed, the binary failure rule
was operationalized and a 60-second policy horizon was added. Their policy
durations were 32.081, 42.657, and 37.124 seconds, so the new horizon does not
reclassify them. The decision timing, empirical basis, exact rules, and
retention of the first three trials are disclosed in
`experiment/evaluation/main_recollection_20260817/protocol_amendment_20260826_failure_criteria_timeout.md`.

## Retrospective inference-loading amendment (2026-08-26)

After 19 eligible smooth-controller trials were complete, the runner changed
from reconstructing one identical RGBActBackend per trial to retaining all 12
frozen backends in one CUDA process and selecting the scheduled blind model
code. Checkpoints, preprocessing, normalization, reset semantics, inference
code, observations, actions, controller, schedule, and outcome rules are
unchanged. The first 19 results are retained. Details and the decision timing
are in
`experiment/evaluation/main_recollection_20260817/protocol_amendment_20260826_resident_model_pool.md`.
