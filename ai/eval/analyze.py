#!/usr/bin/env python3
"""Frozen retrospective raw-rollout metrics and mixed-effects analyses."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from ai.eval.validate_rollout import lift_hold_metrics


def _event_value(events, name):
    if isinstance(events, dict):
        return events.get(name)
    return getattr(events, name, None)


def _pose_matrix(trace):
    values = trace["ee_pose"]
    return np.stack([np.asarray(value, dtype=np.float64) for value in values])


def _interpolate_pose(trace, stamp):
    times = trace["t"].to_numpy(dtype=np.float64)
    poses = _pose_matrix(trace)
    if stamp <= times[0]:
        return poses[0]
    if stamp >= times[-1]:
        return poses[-1]
    upper = int(np.searchsorted(times, stamp, side="right"))
    lower = upper - 1
    weight = (stamp - times[lower]) / (times[upper] - times[lower])
    return poses[lower] * (1.0 - weight) + poses[upper] * weight


def grasp_errors(trace, events, p_target):
    """Return (E1, E2, E3, clean) using only the pre-contact trajectory."""
    if len(trace) == 0:
        raise ValueError("empty trace")
    t_move = _event_value(events, "t_move")
    t_close = _event_value(events, "t_close")
    t_end = _event_value(events, "t_end")
    candidates = [float(x) for x in (t_move, t_close, t_end) if x is not None]
    cutoff = min(candidates) if candidates else float(trace["t"].iloc[-1])
    pre = trace[trace["t"] <= cutoff]
    if len(pre) == 0:
        pre = trace.iloc[[0]]
    target = np.asarray(p_target, dtype=np.float64)
    e3 = float(np.min(np.linalg.norm(_pose_matrix(pre)[:, :2] - target[:2], axis=1)))
    if t_close is None:
        return None, None, e3, None
    pose = _interpolate_pose(trace, float(t_close))
    e1 = float(np.linalg.norm(pose[:2] - target[:2]))
    e2 = float(pose[2] - target[2])
    clean = t_move is None or float(t_close) < float(t_move)
    return e1, e2, e3, bool(clean)


def wilson_interval(successes, total, alpha=0.05):
    from scipy.stats import norm

    if total <= 0:
        return (None, None)
    z = float(norm.ppf(1.0 - alpha / 2.0))
    p = successes / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    radius = z * np.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / denominator
    return float(center - radius), float(center + radius)


def descriptive_success(data, group_columns=("condition",)):
    """Summarize rollout success and expose between-model variation.

    The Wilson interval is a descriptive rollout-level interval.  It is not
    presented as if all repeated rollouts were independent model replicates;
    model-level rates are included separately and the inferential model uses a
    model random intercept.
    """
    rows = []
    columns = list(group_columns)
    for keys, group in data.groupby(columns, dropna=False):
        if not isinstance(keys, tuple):
            keys = (keys,)
        successes = int(group["success"].sum())
        total = int(len(group))
        lower, upper = wilson_interval(successes, total)
        row = {
            **dict(zip(columns, keys)),
            "successes": successes, "total": total,
            "rate": successes / total, "wilson_low": lower, "wilson_high": upper,
        }
        if "model_code" in group:
            model_rates = group.groupby("model_code")["success"].mean()
            row.update({
                "model_count": int(len(model_rates)),
                "model_rate_mean": float(model_rates.mean()),
                "model_rate_sd": (
                    float(model_rates.std(ddof=1)) if len(model_rates) > 1 else None
                ),
            })
        rows.append(row)
    return rows


def evaluation_group(position_id: str) -> str:
    """Separate exact-grid primary trials from unseen-Q secondary trials."""
    try:
        index = int(str(position_id).split("_")[-1])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid evaluation position: {position_id}") from exc
    if 1 <= index <= 4:
        return "exact_grid"
    if 5 <= index <= 8:
        return "q_unseen"
    raise ValueError(f"invalid evaluation position: {position_id}")


def eligibility(meta: dict) -> tuple[bool, str | None]:
    """Return the frozen primary-analysis eligibility decision for a trial."""
    if bool(meta.get("object_move")):
        return False, "object_move"
    if meta.get("failure_code") == "hardware_fault":
        return False, "hardware_fault"
    return True, None


def failure_descriptive(data):
    rows = []
    failures = data[data["success"] == False]  # noqa: E712
    for (condition, code), group in failures.groupby(
            ["condition", "failure_code"], dropna=False):
        rows.append({
            "condition": condition,
            "failure_code": code,
            "count": int(len(group)),
        })
    return rows


def duration_descriptive(data):
    rows = []
    for (condition, group_name), group in data.groupby(
            ["condition", "evaluation_group"], dropna=False):
        values = group["duration_s"].dropna().astype(float)
        rows.append({
            "condition": condition,
            "evaluation_group": group_name,
            "n": int(len(values)),
            "mean_s": float(values.mean()) if len(values) else None,
            "median_s": float(values.median()) if len(values) else None,
            "sd_s": float(values.std(ddof=1)) if len(values) > 1 else None,
        })
    return rows


def validate_analysis_set(data, expected_eligible: int | None = None):
    """Reject retry mistakes that would silently double-count a schedule row."""
    eligible = data[data["analysis_eligible"].astype(bool)]
    counts = eligible["scheduled_rollout_id"].value_counts()
    duplicates = counts[counts > 1]
    if len(duplicates):
        raise ValueError(
            "multiple eligible attempts for scheduled rollout(s): "
            + ", ".join(sorted(duplicates.index.astype(str))))
    if expected_eligible is not None and len(counts) != int(expected_eligible):
        raise ValueError(
            f"expected {expected_eligible} eligible scheduled rollouts, "
            f"found {len(counts)}")


def fit_models(data):
    """Fit the single preregistered factorial model with crossed random intercepts."""
    import statsmodels.formula.api as smf
    from statsmodels.genmod.bayes_mixed_glm import BinomialBayesMixedGLM

    data = data.copy()
    data["success"] = data["success"].astype(int)
    fixed = "C(condition) * d"
    success_model = BinomialBayesMixedGLM.from_formula(
        "success ~ " + fixed,
        {
            "model": "0 + C(model_code)",
            "position": "0 + C(position_id)",
        },
        data,
    ).fit_vb()
    results = {
        "formula": (
            "success/E1/E2/E3 ~ condition * d + (1|model_code) + "
            "(1|position_id)"
        ),
        "success": {
            "fixed_names": list(success_model.model.exog_names),
            "posterior_mean": success_model.fe_mean.tolist(),
            "posterior_sd": success_model.fe_sd.tolist(),
        },
        "continuous": {},
        "success_descriptive": {
            "overall": descriptive_success(data),
            "by_evaluation_group": descriptive_success(
                data, ("condition", "evaluation_group")),
            "by_session": descriptive_success(data, ("condition", "session")),
        },
    }
    for metric in ("E1", "E2", "E3"):
        subset = data.dropna(subset=[metric]).copy()
        if metric in ("E1", "E2") and "clean" in subset:
            subset = subset[subset["clean"].astype(bool)]
        model = smf.mixedlm(
            f"{metric} ~ {fixed}", subset, groups=subset["model_code"],
            vc_formula={
                "position": "0 + C(position_id)",
            }, re_formula="1").fit()
        results["continuous"][metric] = {
            "coefficients": {key: float(value) for key, value in model.params.items()},
            "confidence_intervals": {
                key: [float(bounds.iloc[0]), float(bounds.iloc[1])]
                for key, bounds in model.conf_int().iterrows()
            },
            "n": int(model.nobs),
            "converged": bool(model.converged),
        }
    return results


def load_rollout(directory):
    """Load one finalized rollout into the canonical analysis row."""
    directory = Path(directory)
    meta = json.loads((directory / "meta.json").read_text(encoding="utf-8"))
    events = json.loads((directory / "events.json").read_text(encoding="utf-8"))
    trace = pd.read_parquet(directory / "trace.parquet")
    if meta.get("p_target") is None:
        raise ValueError(f"{directory}: p_target is null")
    e1, e2, e3, clean = grasp_errors(trace, events, meta["p_target"])
    close_pose = None
    if events.get("t_close") is not None and len(trace):
        close_pose = _interpolate_pose(trace, float(events["t_close"]))[:3]
    close_error_3d = None
    if close_pose is not None:
        close_error_3d = float(np.linalg.norm(
            close_pose - np.asarray(meta["p_target"], dtype=np.float64)))
    motion = lift_hold_metrics(
        trace, events.get("t_close"),
        float(meta.get("lift_threshold_mm", 50.0)))
    eligible, exclusion_reason = eligibility(meta)
    duration_s = None
    if events.get("t_start") is not None and events.get("t_end") is not None:
        duration_s = float(events["t_end"]) - float(events["t_start"])
    correction = meta.get("outcome_correction") or {}
    return {
        **meta,
        "attempt_directory": directory.name,
        "grasp_x_mm": None if close_pose is None else float(close_pose[0]),
        "grasp_y_mm": None if close_pose is None else float(close_pose[1]),
        "grasp_z_mm": None if close_pose is None else float(close_pose[2]),
        "target_x_mm": float(meta["p_target"][0]),
        "target_y_mm": float(meta["p_target"][1]),
        "target_z_mm": float(meta["p_target"][2]),
        "E1": e1, "E2": e2, "E3": e3,
        "close_error_3d_mm": close_error_3d, "clean": clean,
        "lift_mm": motion["lift_mm"],
        "measured_hold_duration_s": motion["hold_duration_s"],
        "duration_s": duration_s,
        "evaluation_group": evaluation_group(meta.get("position_id")),
        "analysis_eligible": eligible,
        "exclusion_reason": exclusion_reason,
        "outcome_corrected": bool(correction),
        "original_success": correction.get("original_success"),
        "correction_reason": correction.get("reason"),
    }


def load_rollouts(root):
    rows = [
        load_rollout(meta_path.parent)
        for meta_path in sorted(Path(root).glob("*/meta.json"))
    ]
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--rollouts")
    source.add_argument(
        "--input-table",
        help=(
            "validated per-rollout CSV, normally results/rollouts.csv; "
            "this avoids counting incomplete retry directories"
        ),
    )
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--rollout-table",
        help="per-rollout CSV path (default: <output stem>_rollouts.csv)")
    parser.add_argument("--expected-eligible", type=int, default=480)
    parser.add_argument("--metrics-only", action="store_true")
    args = parser.parse_args()
    data = (
        pd.read_csv(args.input_table)
        if args.input_table
        else load_rollouts(args.rollouts)
    )
    validate_analysis_set(
        data, None if args.metrics_only else int(args.expected_eligible))
    eligible = data[data["analysis_eligible"].astype(bool)].copy()
    exclusions = data[~data["analysis_eligible"].astype(bool)].copy()
    result = {
        "rollouts": data.to_dict(orient="records"),
        "eligibility": {
            "eligible": int(len(eligible)),
            "excluded": int(len(exclusions)),
            "exclusion_reasons": exclusions["exclusion_reason"].value_counts().to_dict(),
        },
        "success_descriptive": {
            "overall": descriptive_success(eligible),
            "by_evaluation_group": descriptive_success(
                eligible, ("condition", "evaluation_group")),
            "by_session": descriptive_success(
                eligible, ("condition", "session")),
        },
        "failure_descriptive": failure_descriptive(eligible),
        "duration_descriptive": duration_descriptive(eligible),
    }
    if not args.metrics_only:
        result["models"] = fit_models(eligible)
    output_path = Path(args.output)
    table_path = (
        Path(args.rollout_table) if args.rollout_table
        else output_path.with_name(output_path.stem + "_rollouts.csv")
    )
    table_path.parent.mkdir(parents=True, exist_ok=True)
    data.to_csv(table_path, index=False)
    result["artifacts"] = {"rollout_table_csv": str(table_path)}
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
