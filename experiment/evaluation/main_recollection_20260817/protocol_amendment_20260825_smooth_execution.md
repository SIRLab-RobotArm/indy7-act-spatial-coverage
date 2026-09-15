# Execution amendment: 30 Hz interpolated Indy targets

Date: 2026-08-25 (KST)

## Trigger and diagnosis

After four scheduled rollouts, the operator reported visible repeated shaking
during policy motion. The deployed ACT endpoint rate was 5 Hz and each
2--5 mm cumulative endpoint was passed directly to Indy `MoveTeleL` every
200 ms. Demonstration teleoperation instead sent approximately 0.5 mm targets
at 30 Hz. Archived rollout actions were inspected before changing the
controller. Trial 4 had only 3.26% consecutive direction reversals and 1.40%
sharp reversals (cosine below -0.5), so policy oscillation was not the primary
explanation. The command-rate mismatch and repeated acceleration toward
coarse endpoints were the most plausible source.

## Frozen change

- ACT inference, chunk size, replanning interval, 5 Hz action endpoints,
  XYZ values, gripper threshold/timing, maximum 25 mm action clamp, and manual
  stop rule remain unchanged.
- Each 5 Hz cumulative Cartesian endpoint is split into six equal linear
  cumulative targets and published at 30 Hz.
- No action scaling, low-pass filtering, endpoint modification, rotation, or
  outcome-dependent tuning is introduced.
- Rollout metadata records `execution_controller=linear_interp_30hz_v1`,
  `policy_action_hz=5.0`, `robot_command_hz=30.0`, and
  `interpolation_steps=6`.

## Evaluation restart

The old output root `experiment/rollouts/main_recollection_20260817_main`
contains four finalized trials under the coarse 5 Hz controller and one empty
Trial 5 startup directory. These artifacts are retained for diagnosis but are
excluded from the final 480-rollout analysis. Mixing controller revisions
would confound condition and seed comparisons.

Repository cleanup on 2026-09-01 moved this diagnostic root without modifying
its contents to
`archive/development_artifacts_20260901/experiment/rollouts/main_recollection_20260817_main`.

After one Condition A/seed 0 hardware pilot at demonstrated G5 confirms
visibly smooth motion, the complete frozen
480-row schedule restarts at Trial 1 without changing its order. New primary
results are written only to:

`experiment/rollouts/main_recollection_20260817_main_smooth30hz`

This is a retrospective operational amendment made after outcomes from the
first four coarse-controller trials were known. It must not be described as a
prospective preregistration.
