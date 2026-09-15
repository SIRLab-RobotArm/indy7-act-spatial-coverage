"""Crash-safe storage for repeated end-effector teach-in samples.

Poses use the experiment convention ``[x, y, z, rx, ry, rz]`` in mm/deg.
Euler angles are averaged after unwrapping around the first sample.  This is
appropriate for the small orientation spread expected during repeated
teach-in, but is not a general SO(3) averaging implementation.
"""

import math
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

import yaml


TEACH_POSITION_IDS = (
    *(f'grid_{index}' for index in range(1, 10)),
    *(f'eval_{index}' for index in range(5, 9)),
)

DISPLAY_NAMES = {
    **{f'grid_{index}': f'G{index}' for index in range(1, 10)},
    **{f'eval_{index + 4}': f'Q{index}' for index in range(1, 5)},
}

MIRROR_TARGETS = {
    'eval_1': 'grid_5',
    'eval_2': 'grid_1',
    'eval_3': 'grid_6',
    'eval_4': 'grid_8',
}


def wrap180(value):
    """Wrap degrees to [-180, 180)."""
    return (float(value) + 180.0) % 360.0 - 180.0


def rotation_flip(sample, reference):
    """Return True when an Euler component differs by more than 90 degrees."""
    return any(
        abs(wrap180(sample[index] - reference[index])) > 90.0
        for index in range(3, 6)
    )


def mean_pose(samples):
    """Average Cartesian values and locally-unwrapped Euler angles."""
    if not samples:
        return None
    result = [
        sum(float(sample[index]) for sample in samples) / len(samples)
        for index in range(3)
    ]
    reference = samples[0]
    for index in range(3, 6):
        unwrapped = [
            float(reference[index]) + wrap180(
                float(sample[index]) - float(reference[index]))
            for sample in samples
        ]
        result.append(wrap180(sum(unwrapped) / len(unwrapped)))
    return result


def pose_spread(samples, mean):
    """Calculate positional and rotational spread around a mean pose."""
    if not samples or mean is None:
        return {
            'std_mm_deg': None,
            'max_dev_mm_deg': None,
            'xyz_rms_mm': None,
            'xyz_max_mm': None,
            'rot_max_dev_deg': None,
        }

    deviations = []
    for sample in samples:
        deviations.append([
            float(sample[index]) - float(mean[index])
            if index < 3
            else wrap180(float(sample[index]) - float(mean[index]))
            for index in range(6)
        ])

    std = [
        math.sqrt(sum(row[index] ** 2 for row in deviations) / len(deviations))
        for index in range(6)
    ]
    max_dev = [
        max(abs(row[index]) for row in deviations)
        for index in range(6)
    ]
    xyz_distances = [
        math.sqrt(sum(value ** 2 for value in row[:3]))
        for row in deviations
    ]
    rotation_distances = [
        math.sqrt(sum(value ** 2 for value in row[3:])) for row in deviations
    ]
    xyz_squared_sum = sum(distance ** 2 for distance in xyz_distances)
    return {
        'std_mm_deg': std,
        'max_dev_mm_deg': max_dev,
        'xyz_rms_mm': math.sqrt(xyz_squared_sum / len(xyz_distances)),
        'xyz_max_mm': max(xyz_distances),
        'rot_max_dev_deg': max(rotation_distances),
    }


class TeachStore:
    """Persist repeated EEF and joint samples to the experiment YAML file."""

    def __init__(self, path, samples_target=3, spread_warn_mm=3.0):
        """Configure the target file, repetition count, and spread warning."""
        self.path = Path(path).expanduser().resolve()
        self.samples_target = int(samples_target)
        self.spread_warn_mm = float(spread_warn_mm)
        if self.samples_target < 1:
            raise ValueError('samples_target must be at least 1')

    def _read(self):
        with self.path.open('r', encoding='utf-8') as stream:
            return yaml.safe_load(stream) or {}

    def _write(self, data):
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
        temporary.replace(self.path)

    @staticmethod
    def _samples(entry):
        teach = entry.get('teach') or {}
        samples = teach.get('samples_mm_deg') or []
        return [[float(value) for value in sample] for sample in samples]

    @staticmethod
    def _joint_samples(entry):
        teach = entry.get('teach') or {}
        samples = teach.get('joint_deg') or []
        return [[float(value) for value in sample] for sample in samples]

    def get_samples(self, position_id):
        """Return the saved EEF samples for one direct-teach position."""
        data = self._read()
        entry = data.get('positions', {}).get(position_id, {}) or {}
        return self._samples(entry)

    def load_counts(self):
        """Return the current EEF sample count for each of the 13 positions."""
        data = self._read()
        positions = data.get('positions', {})
        counts = {}
        for position_id in TEACH_POSITION_IDS:
            entry = positions.get(position_id, {}) or {}
            samples = self._samples(entry)
            source = entry.get('source') or {}
            if (
                    not samples
                    and entry.get('target_defined')
                    and source.get('method') in {
                        'successful_demonstration_first_close_mean',
                        'surrounding_grid_cell_center_mean',
                    }):
                # Main-experiment targets are derived without manual teaching.
                # Mark them complete so MODE cannot accidentally overwrite Q.
                counts[position_id] = self.samples_target
            else:
                counts[position_id] = len(samples)
        return counts

    def _rebuild_entry(self, samples, joint_samples):
        average = mean_pose(samples)
        spread = pose_spread(samples, average)
        joint_mean = None
        if joint_samples:
            joint_count = len(joint_samples)
            joint_mean = [
                sum(sample[index] for sample in joint_samples) / joint_count
                for index in range(6)
            ]
        spread_warning = False
        if spread['xyz_max_mm'] is not None:
            spread_warning = spread['xyz_max_mm'] > self.spread_warn_mm
        entry = {
            'ee_pose_mm_deg': average,
            'taught': len(samples) >= self.samples_target,
            'teach': {
                'n_samples': len(samples),
                'samples_mm_deg': samples,
                'joint_deg': joint_samples,
                'joint_mean_deg': joint_mean,
                **spread,
                'spread_warning': bool(spread_warning),
                'updated_utc': datetime.now(timezone.utc).isoformat(),
            },
        }
        return entry

    @staticmethod
    def _apply_mirrors(data):
        positions = data.setdefault('positions', {})
        for target, source in MIRROR_TARGETS.items():
            source_entry = positions.get(source, {}) or {}
            positions[target] = {
                'ee_pose_mm_deg': source_entry.get('ee_pose_mm_deg'),
                'taught': bool(source_entry.get('taught', False)),
                'mirror_of': source,
            }
            if source_entry.get('teach') is not None:
                positions[target]['teach'] = deepcopy(source_entry['teach'])
            if source_entry.get('source') is not None:
                positions[target]['source'] = deepcopy(source_entry['source'])

    def append_sample(self, position_id, pose, joints):
        """Append one synchronized EEF/joint sample and rebuild its summary."""
        if position_id not in TEACH_POSITION_IDS:
            raise ValueError(f'unsupported teach position: {position_id}')
        if len(pose) != 6:
            raise ValueError('EEF pose must contain 6 values')
        if len(joints) != 6:
            raise ValueError('joint sample must contain 6 values')

        data = self._read()
        positions = data.setdefault('positions', {})
        previous_entry = positions.get(position_id, {}) or {}
        samples = self._samples(previous_entry)
        joint_samples = self._joint_samples(previous_entry)
        if len(joint_samples) != len(samples):
            raise ValueError(
                f'{position_id} has mismatched EEF and joint sample counts')
        if len(samples) >= self.samples_target:
            raise ValueError(
                f'{position_id} already has {self.samples_target} samples')

        sample = [float(value) for value in pose]
        if samples and rotation_flip(sample, samples[0]):
            raise ValueError(
                'orientation differs by more than 90 deg from the first '
                'sample')
        samples.append(sample)
        joint_samples.append([float(value) for value in joints])
        positions[position_id] = self._rebuild_entry(samples, joint_samples)
        self._apply_mirrors(data)
        self._write(data)
        return positions[position_id]

    def pop_sample(self, position_id):
        """Remove the most recent matching EEF and joint sample."""
        if position_id not in TEACH_POSITION_IDS:
            raise ValueError(f'unsupported teach position: {position_id}')
        data = self._read()
        positions = data.setdefault('positions', {})
        previous_entry = positions.get(position_id, {}) or {}
        samples = self._samples(previous_entry)
        joint_samples = self._joint_samples(previous_entry)
        if len(joint_samples) != len(samples):
            raise ValueError(
                f'{position_id} has mismatched EEF and joint sample counts')
        if not samples:
            return None
        removed = samples.pop()
        joint_samples.pop()
        positions[position_id] = self._rebuild_entry(samples, joint_samples)
        self._apply_mirrors(data)
        self._write(data)
        return removed

    def summary(self):
        """Return counts and the number of completed direct-teach positions."""
        counts = self.load_counts()
        return {
            'counts': counts,
            'completed': sum(
                count >= self.samples_target for count in counts.values()
            ),
            'total': len(counts),
        }
