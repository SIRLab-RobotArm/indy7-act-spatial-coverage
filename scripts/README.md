# Entry-point scripts

Runnable entry points for collecting data, training, evaluating and analysing.
The real logic lives in `ai/` and `ros2_ws/src/`; these scripts just assemble the
environment and arguments consistently.

Every wrapper sources [`env.sh`](env.sh), which finds the repository root from its
own location and loads `config/local.env` if you made one. That is why nothing
here contains a hardcoded path. See `config/local.env.example`.

## Layout

| Directory | Role | Main entry points |
| --- | --- | --- |
| `setup/` | Build the Python environment | `create_env.sh` |
| `collection/` | Per-position HDF5 recording | `collect_p1.sh` ... `collect_p9.sh`, `collect_position.sh` |
| `train/` | The A-D x seed 0/1/2 training matrix | `run_main_matrix.sh` |
| `eval/` | Targets, schedule, trial execution, result export | `prepare_targets.py`, `generate_schedule.py`, `evaluation_ui.sh`, `export_results.py` |
| `analysis/` | Result figures | `plot_results.py` |
| `qa/` | Compare grasp positions between collection batches | `compare_grasp_batches.py` |
| `robot/` | Small helpers | `back_home.sh`, `open_gripper.sh`, `move_to_p5.py` |

## Common commands

Set up the Python environment from nothing:

```bash
./scripts/setup/create_env.sh          # add cu130, cu121, ... for a GPU build
```

Redraw the success-rate figure from the published results, no robot needed:

```bash
python scripts/analysis/plot_results.py --split
```

Re-export the 480-trial result tables from the raw rollouts (needs the 640 GB of
traces):

```bash
source scripts/env.sh && grip_activate_conda
python scripts/eval/export_results.py --require-complete
```

## Cautions

`collection/`, `robot/`, `eval/evaluation_ui.sh` and `eval/run_selected_model.sh`
**move real hardware**. Check the target machine, the output path and the safety
state before running any of them.

`run_main_matrix.sh` refuses to overwrite an existing `ai/models/main_recollection_*`
output, so a stray rerun cannot destroy trained models.

Do not add runs to the finished evaluation schedule. New collection, training or
evaluation work belongs under a new experiment ID and a new output path.
