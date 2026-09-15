"""Pure, preregistered action-to-command contract used by the executor."""

from __future__ import annotations

import numpy as np


MAX_TRANSLATION_DELTA_MM = 25.0
GRIPPER_CLOSE_THRESHOLD = 0.6
GRIPPER_OPEN_THRESHOLD = 0.4


def clamp_translation(delta, max_delta_mm: float = MAX_TRANSLATION_DELTA_MM):
    delta = np.asarray(delta, dtype=np.float32).reshape(3)
    norm = float(np.linalg.norm(delta))
    if norm > max_delta_mm:
        delta = delta * (max_delta_mm / norm)
    return delta


def clamp_to_workspace(current_xyz, delta, workspace_min, workspace_max):
    """Clamp the commanded target to the preregistered Cartesian safety box."""
    current = np.asarray(current_xyz, dtype=np.float32).reshape(3)
    lower = np.asarray(workspace_min, dtype=np.float32).reshape(3)
    upper = np.asarray(workspace_max, dtype=np.float32).reshape(3)
    if np.any(lower >= upper):
        raise ValueError("invalid workspace bounds")
    target = np.clip(current + np.asarray(delta, dtype=np.float32), lower, upper)
    return target - current


def executor_command(action, current_xyz=None, workspace_min=None, workspace_max=None,
                     max_delta_mm: float = MAX_TRANSLATION_DELTA_MM):
    value = np.asarray(action, dtype=np.float32).reshape(-1)
    if value.shape != (4,):
        raise ValueError(f"expected 4-D action, got {value.shape}")
    if not np.all(np.isfinite(value)):
        raise ValueError("action contains a non-finite value")
    delta = value[:3]
    if current_xyz is not None:
        if workspace_min is None or workspace_max is None:
            raise ValueError("workspace bounds are required with current_xyz")
        delta = clamp_to_workspace(current_xyz, delta, workspace_min, workspace_max)
    delta = clamp_translation(delta, max_delta_mm)
    pose_delta = np.concatenate((delta, np.zeros(3, dtype=np.float32)))
    return pose_delta, float(value[3])


def accumulate_pose_target(current_target, pose_delta):
    """Integrate one local XYZ action into an Indy task-relative target."""
    target = np.asarray(current_target, dtype=np.float32).reshape(-1)
    step = np.asarray(pose_delta, dtype=np.float32).reshape(-1)
    if target.shape != (6,) or step.shape != (6,):
        raise ValueError("current_target and pose_delta must be 6-D")
    if not np.all(np.isfinite(target)) or not np.all(np.isfinite(step)):
        raise ValueError("pose target contains a non-finite value")
    if np.any(step[3:] != 0):
        raise ValueError("rotation deltas are disabled")
    result = target + step
    result[3:] = 0.0
    return result


class GripperLatch:
    """Trial-scoped binary latch with the preregistered 0.6/0.4 hysteresis."""

    def __init__(self, initial=0):
        self.state = int(bool(initial))
        self.transitioned = False

    def update(self, prediction):
        previous = self.state
        value = float(prediction)
        if self.transitioned:
            return self.state, None
        if self.state == 0 and value > GRIPPER_CLOSE_THRESHOLD:
            self.state = 1
        elif self.state == 1 and value < GRIPPER_OPEN_THRESHOLD:
            self.state = 0
        transition = None if self.state == previous else ("close" if self.state else "open")
        self.transitioned = transition is not None
        return self.state, transition
