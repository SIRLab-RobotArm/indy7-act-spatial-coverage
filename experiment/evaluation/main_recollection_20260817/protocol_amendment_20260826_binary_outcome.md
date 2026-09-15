# Protocol amendment: binary operator outcome and immediate grasp error

Date: 2026-08-26 KST

This amendment was made after the separate 30 Hz controller pilot and before
the new 480-rollout main block began. Therefore no main outcome was observed
under the new result root before this decision.

The real-time operator interface now records only a binary visual outcome:

- `success`: the PVC is grasped, lifted at least 50 mm, and held for at least
  3 seconds;
- `operator_failure`: one or more success criteria are not met.

Legacy detailed failure services remain available to read and validate the
excluded 5 Hz diagnostic records, but the new main runner does not ask the
operator to subtype failures. This reduces subjective classification burden
and keeps the primary outcome aligned with the paper's success-rate analysis.
Equipment faults and external object movement are not labeled as policy
failure: the attempt is aborted and preserved, then the same scheduled trial
is repeated with a new attempt ID.

After every finalized success or failure, the runner prints the frozen target
XYZ, actual first-close EEF XYZ, signed XYZ error, planar XY error, and 3D
distance error. For exact-grid evaluations the target is the successful
demonstration first-close mean. For Q evaluations it is the preregistered
equal-weight mean of the four surrounding grid targets. These values are
computed from the saved trace and events; they do not change policy input,
success labels, target definitions, or the fixed 480-row schedule.

The interactive runner accepts confirmation and outcome words without English
case sensitivity and continues in the same terminal from HOME and OPEN for the
next scheduled trial.

This amendment is supplemented by
`protocol_amendment_20260826_failure_criteria_timeout.md`, which was fixed after
smooth-controller Trials 1--3. It operationalizes the binary failure rule and
adds a 60-second policy horizon without changing those three stored labels.
