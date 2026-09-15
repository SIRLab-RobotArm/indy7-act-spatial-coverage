import numpy as np

from ai.train.run_train import copy_state_baseline


class _NumericColumns:
    def __init__(self, state, action):
        self._values = {
            "observation.state": np.asarray(state, dtype=np.float32),
            "action": np.asarray(action, dtype=np.float32),
        }
        self.selected = None
        self.format = None

    def select_columns(self, columns):
        self.selected = columns
        return self

    def with_format(self, output_format):
        self.format = output_format
        return self

    def __getitem__(self, item):
        assert item == slice(None)
        return self._values


class _Dataset:
    def __init__(self, state, action):
        self.hf_dataset = _NumericColumns(state, action)


def test_copy_state_baseline_reads_only_numeric_columns():
    dataset = _Dataset(
        state=[[0] * 9 + [0], [0] * 9 + [1]],
        action=[[1, -2, 3, 0], [-1, 0, 1, 0]],
    )

    result = copy_state_baseline(dataset)

    assert dataset.hf_dataset.selected == ["observation.state", "action"]
    assert dataset.hf_dataset.format == "numpy"
    assert result["sample_count"] == 2
    np.testing.assert_allclose(result["l1_per_dimension"], [1, 1, 2, 0.5])
    assert result["l1_mean"] == 1.125
