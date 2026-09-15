# Protocol amendment: operational failure criteria and 60-second horizon

Date: 2026-08-26 KST

This retrospective amendment was fixed after the outcomes of smooth-controller
Trials 1--3 were known and before Trial 4. It must not be described as a
prospective preregistration. The stored first three policy-active durations are
32.081, 42.657, and 37.124 seconds, respectively; therefore the 60-second
horizon below does not reclassify any of them. Their binary labels and rollout
files remain unchanged.

## Frozen binary outcome

A rollout is successful only when all of the following are satisfied:

1. the intended upright PVC is securely grasped;
2. EEF Z rises at least 50 mm after the first close; and
3. the PVC remains held without external support or dropping for at least
   3 seconds.

Any of the following is a policy failure and remains in the success-rate
denominator as `success=false`, `failure_code=operator_failure`:

- the predicted motion creates a collision risk with the table, wall, or
  barrier, so the operator stops it before contact;
- actual unintended robot or gripper contact occurs;
- the upright PVC topples so that its side contacts the table or barrier;
- the gripper closes without enclosing the intended PVC or closes at an
  unrelated location;
- the PVC is missed, slips, or is dropped;
- the 50 mm lift or 3-second hold criterion is not achieved; or
- 60 seconds elapse after the successful `/grip_eval/start` response without
  success.

The operator must not wait for a predicted collision to occur. The UI failure
control sends `F`, after which the runner calls `/grip_eval/stop` before writing
the binary result. If danger is immediate, the physical E-stop takes priority
over every software control. A safety intervention caused by policy motion is
still a policy failure, not an equipment-fault exclusion.

Equipment malfunction unrelated to policy behavior and external movement of
the object or board after logger start remain excluded attempts and are rerun
under a new attempt ID. They are not relabeled as policy failures.

## Horizon basis and geometric outcomes

Across the 220 successful training demonstrations, recorded episode duration
had a median of 39.195 seconds, 95th percentile of 52.838 seconds, and maximum
of 59.243 seconds. A common 60-second policy horizon therefore covers the
demonstrated task duration while giving every model, position, and repeat the
same opportunity.

Distance from the frozen target is not a binary failure rule. The robot starts
far from the object, and a successful path may temporarily move away from it.
First-close signed XYZ error, XY distance error, and 3D distance error remain
continuous secondary outcomes. When no close occurs, close error is missing;
pre-stop minimum approach distance can be analyzed separately.

The UI continues to ask only for binary `S` or `F`; it does not require the
operator to subtype failures online. The operational criteria are shown in the
UI, and detailed failure mechanisms may be rescored from the condition-blinded
recorded video sample.
