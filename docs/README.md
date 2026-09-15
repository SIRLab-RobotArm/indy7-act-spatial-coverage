# Documentation

Reference material for the completed Indy7 ACT RGB-only PVC grasping experiment.

| File | What it covers |
| --- | --- |
| [`experiment_design.md`](experiment_design.md) | The design of record: grid layout, episode counts, the A-D training conditions, the 480-trial evaluation |
| [`data_collection_guide.md`](data_collection_guide.md) | The data contract every recorded episode satisfies, and how to collect more |
| [`data_and_weights.md`](data_and_weights.md) | Where the datasets, model weights and raw traces live, and what is published |
| [`manual_teleop.md`](manual_teleop.md) | Driving the arm, hand and camera by hand, with the safety rules |
| Mark7 serial protocol *(not included)* | The manufacturer's document ("Mand.ro Mark 7 Communication Protocol", 2026-03-05) is available from Mand.ro |

Start with the [top-level README](../README.md) if you have not read it yet - it
explains the question, the result, and how to recompute the numbers without a
robot.

The final measurements live in
[`../experiment/evaluation/main_recollection_20260817/results/README.md`](../experiment/evaluation/main_recollection_20260817/results/README.md),
and the training datasets are described in [`../datasets/README.md`](../datasets/README.md).

**When these documents and the frozen artifacts disagree, the artifacts win.**
The schedule CSV, the membership manifests and the recorded results are what
actually happened; prose can drift.

When editing, do not rewrite a finished experiment as though it had been planned
that way from the start. Where reality diverged from the plan, record both: what
was intended, and the protocol amendment that changed it.
