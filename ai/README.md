# AI pipeline

Turns raw RGB-only HDF5 demonstrations into LeRobot v3 datasets, trains ACT
policies on them, and serves those policies to the real robot.

```text
convert/         inspect HDF5, convert to four-phase 5 Hz trajectories, freeze condition subsets
train/           the LeRobot ACT training wrapper and its frozen hyperparameters
serve/           RGB-only ZMQ inference servers: single-model, and the 12-model pool
eval/            rollout validation, grasp error, and the preregistered analysis
lerobot_source/  a pinned copy of LeRobot 0.5.1 (Apache-2.0, see ../NOTICE.md)
models/          the 12 evaluated 100k checkpoints (not in git, too large)
```

## The artifacts this experiment produced

Condition datasets: `datasets/main_recollection_20260817_rgb_{a,b,c,d}`

Final checkpoints, 12 of them:

```text
ai/models/main_recollection_20260817_<condition>_s<seed>_100000/checkpoints/last/pretrained_model
```

Only the final 100k checkpoints are kept. The intermediate 20k-80k checkpoints,
the optimizer `training_state`, the Hugging Face cache and the smoke, overfit and
pilot models were all removed after the experiment finished, because none of them
were used in any published analysis. Retraining from scratch does not need them.

## Running it

```bash
source scripts/env.sh && grip_activate_conda
```

`scripts/env.sh` sets `HF_HOME` to `ai/.cache/huggingface` inside the repository,
so downloaded artifacts stay out of your home directory. The cache directory is
recreated automatically when something needs downloading.

Train the full matrix with `scripts/train/run_main_matrix.sh`, and run the
evaluation with `scripts/eval/evaluation_ui.sh`.

## Where to read next

- The design that produced these artifacts: [`../docs/experiment_design.md`](../docs/experiment_design.md)
- The verified 480-trial results: [`../experiment/evaluation/main_recollection_20260817/results/README.md`](../experiment/evaluation/main_recollection_20260817/results/README.md)
