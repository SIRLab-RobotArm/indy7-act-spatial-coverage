"""Frozen evaluation definitions and deterministic rollout scheduling.

The physical table locations are represented in the Indy base frame.  The
nearest-training-location distance ``d`` is deliberately planar (XY): the
experiment varies the object's table location, while Z is the demonstrated
grasp height and is evaluated separately as E2.
"""

from __future__ import annotations

import random


GRID_IDS = tuple(f"grid_{index}" for index in range(1, 10))
EVAL_IDS = tuple(f"eval_{index}" for index in range(1, 9))
EVAL_TO_GRID = {
    "eval_1": "grid_5",
    "eval_2": "grid_1",
    "eval_3": "grid_6",
    "eval_4": "grid_8",
}
CONDITION_GRIDS = {
    "A": ("grid_5",),
    "B": ("grid_1", "grid_6", "grid_8"),
    "C": GRID_IDS,
    "D": GRID_IDS,
}
FAILURE_CODES = (
    "operator_failure",
    "missed_grasp",
    "slip_drop",
    "no_lift",
    "collision_stop",
    "hardware_fault",
)
LIFT_THRESHOLD_MM = 50.0
HOLD_DURATION_S = 3.0
POLICY_TIMEOUT_S = 60.0
SCHEDULE_REPEATS = 5
TRIALS_PER_SESSION = 24


def target_xyz(positions: dict, position_id: str) -> list[float]:
    """Return one complete evaluation target or raise a useful error."""
    entry = (positions.get("positions") or {}).get(position_id) or {}
    pose = entry.get("ee_pose_mm_deg")
    target_defined = entry.get("target_defined", entry.get("taught", False))
    if not target_defined or not isinstance(pose, list) or len(pose) < 3:
        raise ValueError(f"{position_id} does not have a complete p_target")
    return [float(value) for value in pose[:3]]


def planar_distance_mm(first: list[float], second: list[float]) -> float:
    """Return Euclidean XY distance in the Indy base frame."""
    dx = float(first[0]) - float(second[0])
    dy = float(first[1]) - float(second[1])
    return (dx * dx + dy * dy) ** 0.5


def evaluation_distances(positions: dict) -> dict[str, dict[str, float]]:
    """Calculate d for every condition at every complete evaluation target."""
    grid_targets = {
        grid_id: target_xyz(positions, grid_id) for grid_id in GRID_IDS
    }
    distances = {}
    for eval_id in EVAL_IDS:
        evaluation_target = target_xyz(positions, eval_id)
        distances[eval_id] = {
            condition: min(
                planar_distance_mm(evaluation_target, grid_targets[grid_id])
                for grid_id in training_grids
            )
            for condition, training_grids in CONDITION_GRIDS.items()
        }
    return distances


def _shuffle_session(
        rows: list[dict], rng: random.Random, previous_row: dict | None = None
) -> list[dict]:
    """Shuffle one balanced session while avoiding obvious adjacent repeats."""
    by_condition = {
        condition: [row for row in rows if row["condition"] == condition]
        for condition in CONDITION_GRIDS
    }
    for _ in range(1_000):
        # Six independently shuffled A/B/C/D blocks make condition balance
        # explicit and guarantee unlike neighboring conditions.
        condition_order = []
        previous = None
        for _block in range(6):
            block = list(CONDITION_GRIDS)
            rng.shuffle(block)
            if previous is not None and block[0] == previous:
                swap = next(index for index, value in enumerate(block) if value != previous)
                block[0], block[swap] = block[swap], block[0]
            condition_order.extend(block)
            previous = block[-1]

        pools = {condition: list(values) for condition, values in by_condition.items()}
        for pool in pools.values():
            rng.shuffle(pool)
        candidate = []
        for condition in condition_order:
            pool = pools[condition]
            valid = [
                index for index, row in enumerate(pool)
                if not candidate or row["position_id"] != candidate[-1]["position_id"]
            ]
            if not valid:
                break
            candidate.append(pool.pop(rng.choice(valid)))
        if len(candidate) == len(rows) and (
                previous_row is None
                or (
                    previous_row["condition"] != candidate[0]["condition"]
                    and previous_row["model_code"] != candidate[0]["model_code"]
                    and previous_row["position_id"] != candidate[0]["position_id"]
                )):
            return candidate
    raise RuntimeError("could not construct a non-adjacent evaluation session")


def build_balanced_schedule(
        seed: int = 20260824, repeat_count: int = SCHEDULE_REPEATS) -> list[dict]:
    """Build a balanced schedule with five repeats by default.

    The original repeat-1/2 schedule remains the exact 192-trial prefix so the
    three completed main rollouts retain their frozen IDs.  Repeat 3 and later
    are appended in four-session blocks.  Every 24-trial session contains each
    model twice, each condition six times, each seed eight times, and each
    position three times.  Globally every model-position-repeat occurs once.
    """
    repeat_count = int(repeat_count)
    if repeat_count < 2:
        raise ValueError("repeat_count must be at least 2")
    rng = random.Random(int(seed))
    models = [
        {"condition": condition, "seed": model_seed}
        for condition in CONDITION_GRIDS
        for model_seed in range(3)
    ]
    codes = [f"M{index:02d}" for index in range(1, len(models) + 1)]
    rng.shuffle(codes)
    for model, code in zip(models, codes):
        model["model_code"] = code

    schedule = []

    # Preserve the already-frozen 192-trial prefix exactly.
    for session_index in range(8):
        session_rows = []
        for model_index, model in enumerate(models):
            for repeat_offset, repeat in enumerate((1, 2)):
                position_index = (
                    session_index + 2 * model_index + repeat_offset
                ) % len(EVAL_IDS)
                session_rows.append({
                    **model,
                    "session": session_index + 1,
                    "position_id": EVAL_IDS[position_index],
                    "repeat": repeat,
                })
        schedule.extend(_shuffle_session(
            session_rows, rng, schedule[-1] if schedule else None))

    # One repeat needs eight positions/model.  Place two positions/model in
    # each 24-trial session, so four sessions complete one additional repeat.
    next_session = 9
    for repeat in range(3, repeat_count + 1):
        for local_session in range(4):
            session_rows = []
            for model_index, model in enumerate(models):
                for position_offset in range(2):
                    position_index = (
                        2 * local_session + 2 * model_index + position_offset
                    ) % len(EVAL_IDS)
                    session_rows.append({
                        **model,
                        "session": next_session,
                        "position_id": EVAL_IDS[position_index],
                        "repeat": repeat,
                    })
            schedule.extend(_shuffle_session(
                session_rows, rng, schedule[-1] if schedule else None))
            next_session += 1

    for trial_index, row in enumerate(schedule, start=1):
        row["trial"] = trial_index
        row["rollout_id"] = (
            f"t{trial_index:03d}_{row['model_code'].lower()}_"
            f"{row['position_id']}_r{row['repeat']}"
        )
    return schedule
