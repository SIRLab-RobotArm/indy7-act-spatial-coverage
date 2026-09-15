#!/usr/bin/env python3
"""Print the three-terminal commands for one frozen evaluation trial."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import shlex
import sys

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ai.eval.protocol import (
    HOLD_DURATION_S,
    LIFT_THRESHOLD_MM,
    POLICY_TIMEOUT_S,
    target_xyz,
)


DEFAULT_SCHEDULE = Path(
    "experiment/evaluation/main_recollection_20260817/schedule_full.csv")
DEFAULT_POSITIONS = Path("experiment/positions.yaml")
DEFAULT_OUTPUT_ROOT = (
    REPO_ROOT / "experiment/rollouts/main_recollection_20260817_main_smooth30hz")


@dataclass(frozen=True)
class TrialSpec:
    trial: int
    trial_count: int
    attempt: int
    session: int
    model_code: str
    condition: str
    seed: int
    position_id: str
    repeat: int
    scheduled_rollout_id: str
    rollout_id: str
    p_target: list[float]
    distance_mm: float
    metadata: dict
    model: Path
    dataset: Path
    dataset_repo_id: str
    output_root: Path


def trial_spec(
    schedule: Path,
    positions_path: Path,
    trial: int,
    attempt: int = 1,
) -> TrialSpec:
    """Resolve one frozen schedule row into all runtime inputs."""
    if int(attempt) < 1:
        raise ValueError("attempt must be at least 1")
    row, trial_count = _row(schedule, trial)
    positions = yaml.safe_load(positions_path.read_text(encoding="utf-8")) or {}
    position_id = row["position_id"]
    p_target = target_xyz(positions, position_id)
    distances = positions.get("evaluation_distances_mm") or {}
    try:
        distance = float(distances[position_id][row["condition"]])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "evaluation distances are incomplete; rerun "
            "scripts/eval/prepare_targets.py"
        ) from exc

    condition = row["condition"]
    condition_lower = condition.lower()
    seed = int(row["seed"])
    scheduled_rollout_id = row["rollout_id"]
    rollout_id = (
        scheduled_rollout_id if int(attempt) == 1
        else f"{scheduled_rollout_id}_retry{int(attempt) - 1}"
    )
    metadata = {
        "condition": condition,
        "seed": seed,
        "position_id": position_id,
        "repeat": int(row["repeat"]),
        "model_code": row["model_code"],
        "session": int(row["session"]),
        "attempt": int(attempt),
        "scheduled_rollout_id": scheduled_rollout_id,
        "p_target": p_target,
        "d": distance,
        "object_move": False,
        "lift_threshold_mm": LIFT_THRESHOLD_MM,
        "hold_duration_s": HOLD_DURATION_S,
        "policy_timeout_s": POLICY_TIMEOUT_S,
        "experiment_tag": "main_recollection_20260817_main_smooth30hz",
        "execution_controller": "linear_interp_30hz_v1",
        "policy_action_hz": 5.0,
        "robot_command_hz": 30.0,
        "interpolation_steps": 6,
        "inference_server": "resident_12_model_pool_v1",
    }
    return TrialSpec(
        trial=int(row["trial"]),
        trial_count=trial_count,
        attempt=int(attempt),
        session=int(row["session"]),
        model_code=row["model_code"],
        condition=condition,
        seed=seed,
        position_id=position_id,
        repeat=int(row["repeat"]),
        scheduled_rollout_id=scheduled_rollout_id,
        rollout_id=rollout_id,
        p_target=p_target,
        distance_mm=distance,
        metadata=metadata,
        model=REPO_ROOT / (
            f"ai/models/main_recollection_20260817_{condition_lower}_s{seed}_100000/"
            "checkpoints/last/pretrained_model"
        ),
        dataset=REPO_ROOT / f"datasets/main_recollection_20260817_rgb_{condition_lower}",
        dataset_repo_id=f"sirlab/grip_recollection_20260817_{condition_lower}",
        output_root=DEFAULT_OUTPUT_ROOT,
    )


def _row(schedule: Path, trial: int) -> tuple[dict, int]:
    with schedule.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    matches = [row for row in rows if int(row["trial"]) == int(trial)]
    if len(matches) != 1:
        raise ValueError(f"trial {trial} is not unique in {schedule}")
    return matches[0], len(rows)


def commands(schedule: Path, positions_path: Path, trial: int, attempt: int = 1) -> str:
    spec = trial_spec(schedule, positions_path, trial, attempt)
    meta_json = shlex.quote(json.dumps(spec.metadata, separators=(",", ":")))
    root = REPO_ROOT.as_posix()
    output_root = spec.output_root.as_posix()
    return f"""Trial {spec.trial}/{spec.trial_count} | attempt {spec.attempt} | session {spec.session} | {spec.model_code} | {spec.position_id} | repeat {spec.repeat}

[terminal 1: resident ACT model pool]
cd {root}
source scripts/env.sh && grip_activate_conda
python -m ai.serve.zmq_act_pool_server \\
  --model-key experiment/evaluation/main_recollection_20260817/model_key.json \\
  --repo-root {root} \\
  --device cuda

[terminal 2: robot/evaluation graph]
cd {root}
source scripts/env.sh && grip_source_ros
ros2 launch grip_bringup eval.launch.py \\
  rollout_id:={spec.rollout_id} \\
  output_root:={output_root} \\
  show_camera:=true \\
  require_complete_metadata:=true \\
  meta_json:={meta_json}

[terminal 3: operator]
cd {root}
source scripts/env.sh && grip_source_ros
python -m ai.serve.select_act_pool_model {spec.model_code}
./scripts/robot/back_home.sh
ros2 service call /gripper/open std_srvs/srv/Trigger "{{}}"
ros2 service call /grip_eval/log/start std_srvs/srv/Trigger "{{}}"
ros2 service call /grip_eval/start std_srvs/srv/Trigger "{{}}"

# Outcome decided: stop policy first, then choose exactly one result service.
ros2 service call /grip_eval/stop std_srvs/srv/Trigger "{{}}"
ros2 service call /grip_eval/result/success std_srvs/srv/Trigger "{{}}"
# Binary visual outcome: use operator_failure for every policy failure.
# Hardware faults should abort the attempt rather than be labeled as policy failure.
# ros2 service call /grip_eval/result/failure/operator_failure std_srvs/srv/Trigger "{{}}"
# If the PVC was externally moved after logger start:
# ros2 service call /grip_eval/object_move std_srvs/srv/Trigger "{{}}"
ros2 service call /grip_eval/log/stop std_srvs/srv/Trigger "{{}}"
ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop "{{}}"
# After trace and bag finalization, support the PVC and release it before HOME.
ros2 service call /gripper/open std_srvs/srv/Trigger "{{}}"

# Stop terminal 2 cleanly, then validate:
# conda activate act
# python -m ai.eval.validate_rollout {output_root}/{spec.rollout_id}
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", required=True, type=int)
    parser.add_argument(
        "--attempt", type=int, default=1,
        help="1 for the scheduled trial; use 2+ only for a frozen-rule replacement",
    )
    parser.add_argument(
        "--schedule", type=Path,
        default=DEFAULT_SCHEDULE,
    )
    parser.add_argument(
        "--positions", type=Path, default=DEFAULT_POSITIONS,
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        print(commands(args.schedule, args.positions, args.trial, args.attempt))
    except ValueError as exc:
        raise SystemExit(f"ERROR: {exc}") from None
