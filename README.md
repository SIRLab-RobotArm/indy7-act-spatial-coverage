# Does *where* you demonstrate matter more than *how much*?

This repository holds the code, the protocol and the results for a real-robot
study of one question:

> When you teach a robot arm to pick something up by showing it examples, is it
> better to give it **more examples**, or examples spread over **more places**?

We taught a robot to grasp a vertical PVC pipe, using nothing but a single fixed
camera looking down at the table. Then we changed *where on the table* the
teaching examples came from, retrained, and measured how often the robot
succeeded. 480 real grasps later, the answer is clear: **spread beats volume**,
and a policy trained at one spot barely works anywhere else.

| Training condition | What it was taught | Successes | Success rate | 95% CI |
| --- | --- | ---: | ---: | ---: |
| **A** | 60 demos, all at **one** spot | 15 / 120 | 12.5% | 7.7-19.6% |
| **B** | 60 demos, spread over **3** spots | 74 / 120 | 61.7% | 52.7-69.9% |
| **C** | 60 demos, spread over **9** spots | 96 / 120 | 80.0% | 72.0-86.2% |
| **D** | 180 demos, spread over **9** spots | 116 / 120 | 96.7% | 91.7-98.7% |

A, B and C all saw the *same number* of demonstrations. Only the spread changed,
and success went from 12.5% to 80%. Tripling the data on top of that spread
(C to D) added the last 17 points.

The gap is starker at positions the robot was **never** trained on:

| Condition | At trained positions | At new, unseen positions |
| --- | ---: | ---: |
| A | 25.0% | **0.0%** |
| B | 78.3% | 45.0% |
| C | 86.7% | 73.3% |
| D | 96.7% | 96.7% |

Condition A never once succeeded at a position it had not seen. Condition D was
just as good at new positions as at familiar ones.

---

## Table of contents

- [Try it yourself in 10 minutes (no robot needed)](#try-it-yourself-in-10-minutes-no-robot-needed)
- [How the experiment worked](#how-the-experiment-worked)
- [What is in this repository](#what-is-in-this-repository)
- [Reproducing the whole thing (robot required)](#reproducing-the-whole-thing-robot-required)
- [Data and model weights](#data-and-model-weights)
- [Troubleshooting](#troubleshooting)
- [Citing this work](#citing-this-work)
- [License](#license)

---

## Try it yourself in 10 minutes (no robot needed)

The 480 recorded trials ship with this repository as a plain CSV file. You can
recompute every number above on your own laptop. You do **not** need a robot, a
GPU, or the 90 GB of raw video.

### Step 1 - get the code

```bash
git clone https://github.com/SIRLab-RobotArm/indy7-act-spatial-coverage.git
cd indy7-act-spatial-coverage
```

### Step 2 - build the Python environment

You need [Miniconda](https://docs.conda.io/en/latest/miniconda.html) installed.
Then run one script:

```bash
./scripts/setup/create_env.sh
conda activate act
```

That creates a Python 3.12 environment called `act`, installs a CPU build of
PyTorch, installs the packages in `requirements.txt`, and installs the copy of
[LeRobot](https://github.com/huggingface/lerobot) bundled in `ai/lerobot_source/`.
It takes a few minutes and produces an environment of about 2.6 GB.

Already have a conda environment named `act`? The script reuses it rather than
overwriting it. Set `GRIP_CONDA_ENV` in `config/local.env` to use another name.

*Have an NVIDIA GPU and want to train models later?* Pass your CUDA version
instead: `./scripts/setup/create_env.sh cu130`. Run `nvidia-smi` to see which
one your driver supports.

### Step 3 - check that it works

```bash
pytest
```

Expected on a fresh clone without ROS 2: `70 passed, 9 skipped`. The skipped
tests need either ROS 2 or the trained model weights, and each one prints which.
With ROS 2 sourced, the Indy7 driver cloned and the workspace built (see
[Build the ROS 2 workspace](#build-the-ros-2-workspace)), and the weights
present, all `100 passed`.

### Step 4 - recompute the published results

```bash
python -m ai.eval.analyze \
  --input-table experiment/evaluation/main_recollection_20260817/results/rollouts.csv \
  --output /tmp/analysis.json \
  --expected-eligible 480 \
  --metrics-only
```

This reads all 480 trial records, checks that exactly 480 of them are eligible
for analysis, and writes the descriptive statistics - success rates, confidence
intervals, grasp errors, durations - to `/tmp/analysis.json`. It is deterministic:
run it twice and you get byte-identical output.

Drop `--metrics-only` to also fit the mixed-effects models from the analysis
plan. Be aware that this part is **not** reproducible run to run: the Bayesian
fit (`fit_vb`) does not converge on this dataset, and lands on different
coefficients each time, occasionally collapsing to all zeros. It prints
`ConvergenceWarning` when this happens. The descriptive results above, which are
where the headline table comes from, are pure counting and are unaffected.
See [the results README](experiment/evaluation/main_recollection_20260817/results/README.md#known-limitation-of-the-inferential-model).

### Step 5 - redraw the figure

```bash
python scripts/analysis/plot_results.py --split
```

It prints the per-condition breakdown and writes `figures/success_rate.pdf`:

```
group               condition    successes     rate          95% CI
Unseen positions    A              0/60        0.0%     0.0-6.0
Unseen positions    B             27/60       45.0%    33.1-57.5
Unseen positions    C             44/60       73.3%    61.0-82.9
Unseen positions    D             58/60       96.7%    88.6-99.1
Trained positions   A             15/60       25.0%    15.8-37.2
Trained positions   B             47/60       78.3%    66.4-86.9
Trained positions   C             52/60       86.7%    75.8-93.1
Trained positions   D             58/60       96.7%    88.6-99.1
```

Those eight rows are the whole result of the paper, regenerated from the raw
trial records on your own machine. They match the published
`experiment/evaluation/main_recollection_20260817/results/summary.json`.

---

## How the experiment worked

### The setup

A **Neuromeka Indy7** six-axis arm with a **Mand.ro Mark7** robotic hand stands
over a table. A single **Intel RealSense D435** camera is bolted above the table,
pointing down. The robot's only sight of the world is that camera's colour image
at 640x480. We never use the camera's depth sensor - colour pixels only, which
makes the setup cheap to copy.

The object is always the same: a 25 cm length of rigid PVC-U pipe, 2.5 cm across,
standing upright on the table.

### The grid

Nine marked positions form a 3x3 grid on the table, about 15 cm apart
horizontally and 13 cm vertically. We call them G1 to G9, with G5 in the middle.
Four extra positions, Q1 to Q4, sit exactly between the grid points:

```
G1 ----- G2 ----- G3
 |   Q1   |   Q2   |
G4 ----- G5 ----- G6
 |   Q3   |   Q4   |
G7 ----- G8 ----- G9
```

The Q positions matter: **no demonstration was ever recorded there**, and the
robot was never manually taught them. They are the test of whether a policy
learned something general or just memorised specific spots. They sit *inside*
the grid, so this is interpolation, not a demand for wild extrapolation.

### The demonstrations

A human drove the arm with an Xbox controller through 220 successful grasps:
20 at each of the nine grid positions, plus 40 extra at G5 (60 there in total).
Each recording runs from the home pose through approach, grasp, and lift, at
about 20 frames per second.

### The four conditions

From those 220 demonstrations we built four training sets. The exact list of
episodes in each was frozen - written down and checksummed - *before* any model
was trained, so the split could not be quietly adjusted afterwards.

| Condition | Where the demos came from | Total demos | The question it answers |
| --- | --- | ---: | --- |
| A | G5 only, 60 of them | 60 | What if you only ever teach one spot? |
| B | G1, G6, G8 - 20 each | 60 | Same budget, spread over 3 spots |
| C | All nine, 6-7 each | 60 | Same budget, spread over 9 spots |
| D | All nine, 20 each | 180 | Same spread, three times the data |

A, B and C hold the amount of data fixed and vary only the spread. C and D hold
the spread fixed and vary only the amount.

### The models

Each condition was used to train an [ACT](https://tonyzhaozh.github.io/aloha/)
policy (Action Chunking Transformer) three times, with random seeds 0, 1 and 2 -
so a lucky or unlucky initialisation cannot be mistaken for a real effect. That
is **4 conditions x 3 seeds = 12 models**, each trained for exactly 100,000
optimisation steps.

### The trials

Every model was tested at 8 positions - the four grid positions G5, G1, G6, G8,
and the four never-demonstrated positions Q1 to Q4 - five times each.

```
12 models x 8 positions x 5 repeats = 480 trials
```

The models were referred to by blind codes (M01 to M12) during the runs, so the
operator scoring each grasp did not know which condition they were watching. The
mapping is in `experiment/evaluation/main_recollection_20260817/model_key.json`.

A trial counts as a **success** when the robot lifts the pipe at least 50 mm and
holds it for 3 seconds. The policy gets 60 seconds. Anything else is a failure.
If something outside the policy's control went wrong - the pipe was knocked over
by a person, or the hardware faulted - the trial was rerun under a new attempt ID
and the interference was logged rather than silently dropped.

The full running order of all 480 trials was generated from a fixed random seed
and frozen before the first trial. It is in `schedule_full.csv`, next to a
`manifest.json` that records its SHA-256.

### Honest caveats

Read `experiment/preregistration.md` for the full analysis plan, including the
deviations we are disclosing rather than hiding:

- The analysis plan was written down at freeze time but was **not** publicly
  preregistered before data collection, so it is a retrospective plan.
- Demonstrations were collected position by position in blocks, not randomly
  interleaved. Time of day, operator fatigue and lighting drift are therefore
  entangled with position.
- The repeat count was raised from 2 to 5 after the first three trials had been
  run. That change, and four others, are documented in the
  `protocol_amendment_*.md` files, each stating what changed and when.
- One object, one camera, one lighting setup. Nothing here says how these
  policies behave under a different object or a moved camera.

---

## What is in this repository

```text
ai/convert/       turn raw HDF5 recordings into LeRobot training datasets
ai/train/         ACT training wrapper and the frozen hyperparameters
ai/serve/         the policy server that answers the robot at 5 Hz
ai/eval/          trial validation and the preregistered statistical analysis
ai/lerobot_source/  a pinned copy of LeRobot 0.5.1 (Apache-2.0, see NOTICE.md)

ros2_ws/src/grip_collect/   records synchronised demonstrations to HDF5
ros2_ws/src/grip_eval/      runs a policy on the real arm and logs the trial
ros2_ws/src/grip_bringup/   launch files that wire the whole robot graph together
ros2_ws/src/indy7_ros2/     (not included) clone the Indy7 driver here, see below
ros2_ws/src/indy7_teleop/   Xbox controller teleoperation
ros2_ws/src/mark7/          Mand.ro Mark7 hand driver and presets

scripts/setup/      build the Python environment
scripts/collection/ record demonstrations, one script per grid position
scripts/train/      the 12-model training matrix
scripts/eval/       prepare targets, generate the schedule, run trials, export results
scripts/analysis/   redraw the result figure
scripts/robot/      small helpers: go home, open the gripper

experiment/         frozen positions, membership manifests, schedule, results
config/             machine-specific settings (copy local.env.example to start)
docs/               experiment design, data contract, hardware notes
figures/            figures used in the paper
tests/              98 regression tests, no robot required
```

Four directories are deliberately **not** in git because of their size:
`episodes/` (90 GB of raw recordings), `datasets/` (39 GB of converted training
data), `ai/models/` (2.5 GB of weights) and `experiment/rollouts/` (640 GB of
trial traces and ROS bags). Each has a README explaining what belongs there. See
[Data and model weights](#data-and-model-weights).

### Where to read next

1. [`docs/experiment_design.md`](docs/experiment_design.md) - the design of record
2. [`experiment/evaluation/main_recollection_20260817/results/README.md`](experiment/evaluation/main_recollection_20260817/results/README.md) - the final numbers
3. [`experiment/preregistration.md`](experiment/preregistration.md) - hypotheses, analysis plan, disclosed deviations
4. [`docs/data_collection_guide.md`](docs/data_collection_guide.md) - the data contract, and how to collect more
5. [`docs/data_and_weights.md`](docs/data_and_weights.md) - what is published where
6. [`docs/manual_teleop.md`](docs/manual_teleop.md) - driving the hardware by hand

---

## Reproducing the whole thing (robot required)

Everything from here needs physical hardware. **A six-axis industrial arm can
injure you.** Keep the workspace clear, keep an emergency stop within reach, and
never run these commands without watching the robot.

### What you need

| Part | What we used |
| --- | --- |
| Robot arm | Neuromeka Indy7 |
| Hand | Mand.ro Mark7, on USB serial |
| Camera | Intel RealSense D435, fixed above the table facing down |
| Object | 25 cm rigid PVC-U pipe, 2.5 cm outer diameter |
| Controller | Xbox controller, for recording demonstrations |
| Computer | Ubuntu 24.04, ROS 2 Jazzy, an NVIDIA GPU with about 8 GB free |

### Configure your machine

Every path, IP address and device serial lives in one file, so you never have to
edit code:

```bash
cp config/local.env.example config/local.env
${EDITOR:-nano} config/local.env
```

Fill in your robot's IP, your camera's serial number (`rs-enumerate-devices -s`)
and your hand's serial port (`ls -l /dev/serial/by-id/`). Every script reads
this file automatically. It is git-ignored, so your local details stay local.

### Get the Indy7 driver

The Neuromeka Indy7 driver, its interface definitions and the robot description
are **not** part of this repository, because Neuromeka's
[indy-ros2](https://github.com/neuromeka-robotics/indy-ros2) ships without a
license file. The version used in the experiment - upstream `humble-indyDCP3`
at `95a5a9c` plus the changes the experiment needs - is kept in a fork. Clone
it into the workspace:

```bash
git clone --branch indy7-act-spatial-coverage-v1 --depth 1 \
  https://github.com/SIRLab-RobotArm/indy-ros2.git ros2_ws/src/indy7_ros2
```

`SIRLAB_CHANGES.md` in the fork lists every change against upstream. The path
`ros2_ws/src/indy7_ros2/` is git-ignored here, so the driver never ends up in
this repository. The MIT license of this repository does not cover it; see
[`NOTICE.md`](NOTICE.md).

### Build the ROS 2 workspace

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-skip indy_gazebo indy_moveit
source install/setup.bash
cd ..
```

`indy_gazebo` and `indy_moveit` come with the Neuromeka fork but are not used
here, and need Gazebo and MoveIt installed, so they are skipped.

### The ten steps, in order

Each step consumes what the previous one produced. Steps 1-5 are only needed if
you are collecting fresh data; if you obtained our released dataset and weights,
you can start at step 6.

| # | What it does | Command | Roughly how long |
| --- | --- | --- | --- |
| 1 | Record demonstrations at position N | `./scripts/collection/collect_p1.sh` | days |
| 2 | Convert HDF5 to a LeRobot dataset | `python -m ai.convert.build_dataset ...` | ~1 h |
| 3 | Freeze which episodes go in A-D | `python -m ai.convert.freeze_conditions ...` | seconds |
| 4 | Materialise the four training sets | `python -m ai.convert.subset_dataset ...` | ~1 h |
| 5 | Train all 12 models | `./scripts/train/run_main_matrix.sh` | days on one GPU |
| 6 | Compute the target pose of every position | `python scripts/eval/prepare_targets.py` | seconds |
| 7 | Generate the frozen 480-trial schedule | `python scripts/eval/generate_schedule.py` | seconds |
| 8 | Run the trials | `./scripts/eval/evaluation_ui.sh` | weeks |
| 9 | Export the compact result tables | `python scripts/eval/export_results.py --require-complete` | ~1 min |
| 10 | Run the analysis | `python -m ai.eval.analyze ...` | seconds |

The exact arguments used for the published run are recorded inside each
manifest, under the `argv` key - for example
`experiment/manifests/main_recollection_20260817_rgb_all.json`. That is the
authoritative record of what was actually run, not this table.

Two details worth knowing before step 8:

- Step 6 must print `evaluation_distances_ready: true`. If it does not, the
  target positions are incomplete and the schedule cannot be trusted.
- The evaluation loads all 12 models into GPU memory at once, in a single
  process, and switches between them per trial. This costs about 2.5 GB and
  avoids reloading a model for every trial. The design note is in
  `protocol_amendment_20260826_resident_model_pool.md`.

### Running a single model by hand

To watch one model attempt one position, without recording anything:

```bash
./scripts/eval/run_selected_model.sh --model M01 --position G5
```

Or open the small operator GUI:

```bash
./scripts/eval/model_run_ui.sh
```

To see the exact three-terminal command sequence for any scheduled trial:

```bash
python scripts/eval/trial_commands.py --trial 42
```

---

## Data and model weights

Only small files live in git. The heavy artefacts are:

| Artefact | Size | Status |
| --- | ---: | --- |
| Raw HDF5 demonstrations (220 episodes) | 90 GB | Hugging Face release planned |
| Combined LeRobot dataset | 15 GB | Hugging Face release planned |
| Per-condition datasets (A-D) | 24 GB | regenerable from the above |
| The 12 trained models | 2.5 GB | Hugging Face release planned |
| Raw trial traces and ROS bags | 640 GB | archived locally, available on request |
| **The 480-trial result tables** | **a few hundred KB** | **in this repository** |

The Hugging Face upload is planned but not yet done, so there are no links here
yet. Until then, contact <yuykim14@gmail.com> for access.

What is already here is enough to check every published number:
`experiment/evaluation/main_recollection_20260817/results/` holds one row per
trial, the full attempt audit trail including reruns and corrections, and the
summary statistics. See [`docs/data_and_weights.md`](docs/data_and_weights.md).

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'rclpy'`**
The robot-side code needs ROS 2. Source it, and the built workspace, before
running: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`.
If you only want the analysis, ignore it - those tests skip themselves.

**`ModuleNotFoundError: No module named 'indy_interfaces'`**
ROS 2 is sourced but this repository's workspace is not. Run
`source ros2_ws/install/setup.bash`, after building it with `colcon build`.
If `colcon build` itself cannot find `indy_interfaces`, the Indy7 driver is not
cloned yet - see [Get the Indy7 driver](#get-the-indy7-driver).

**`colcon build` fails with `No module named 'catkin_pkg'` or `No module named 'em'`**
A conda Python is ahead of the system Python on your `PATH`, and CMake picked it
up. Run `conda deactivate` (repeat until no environment is active), then delete
`ros2_ws/build`, `ros2_ws/install` and `ros2_ws/log` and build again.

**`ModuleNotFoundError: No module named 'h5py'` from a ROS node**
ROS nodes run under the system Python, not the conda environment. Install h5py,
pygame and pyyaml for the system Python, or point `GRIP_ROS_PYDEPS` in
`config/local.env` at a directory that has them.

**`Address already in use` on port 5557**
A policy server is already running. Find it with `pgrep -af zmq_act`, stop it, or
change `GRIP_ACT_ENDPOINT` in `config/local.env` to a free port.

**The camera does not open, or opens the wrong one**
Check the serial number: `rs-enumerate-devices -s`, then set `GRIP_CAMERA_SERIAL`
in `config/local.env` (keep the leading underscore). Close RealSense Viewer and
any other launch file first - only one process can own the camera.

**`ERROR: no graphical DISPLAY`**
The operator GUIs are Tk windows, so they need a desktop session. Run them from
the machine's own screen, not a plain SSH shell.

**`pytest` tries to collect LeRobot's own tests**
You are running an old checkout. `pyproject.toml` scopes pytest to `tests/`;
make sure you are in the repository root.

**Training refuses to start: "Refusing to overwrite existing output"**
`run_main_matrix.sh` will not overwrite finished models. Move or delete the
directory under `ai/models/` if you really mean to retrain.

---

## Citing this work

If you use this code, protocol or result table, please cite the repository. See
[`CITATION.cff`](CITATION.cff), which GitHub renders as a ready-made citation.
The paper DOI will be added there once it is published.

## License

MIT, see [`LICENSE`](LICENSE). Bundled third-party components (the vendored
LeRobot copy and the Mark7 hand description) keep their own licenses - see
[`NOTICE.md`](NOTICE.md). The Neuromeka Indy7 driver is deliberately not
included; it is fetched separately and is not covered by this repository's
license.
