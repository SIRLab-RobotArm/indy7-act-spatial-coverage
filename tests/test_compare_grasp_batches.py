from pathlib import Path

import h5py
import numpy as np

from scripts.qa.compare_grasp_batches import compare_batches, complete_h5_files


def write_episode(
    path: Path,
    close_xyz,
    *,
    position_id="grid_5",
    gripper=None,
    jitter=False,
):
    frame_count = 7
    stamp = np.arange(frame_count, dtype=np.float64) / 20.0
    state_stamp = stamp.copy()
    if jitter:
        state_stamp[3:] += 0.05
    ee_pose = np.zeros((frame_count, 6), dtype=np.float32)
    ee_pose[2, :3] = np.asarray(close_xyz, dtype=np.float32)
    if gripper is None:
        gripper = [0, 0, 1, 1, 1, 1, 1]
    with h5py.File(path, "w") as output:
        output.create_dataset(
            "/obs/rgb", data=np.zeros((frame_count, 4, 5, 3), dtype=np.uint8),
            compression="lzf",
        )
        output.create_dataset("/state/ee_pose", data=ee_pose)
        output.create_dataset(
            "/state/joint_pos", data=np.zeros((frame_count, 6), dtype=np.float32)
        )
        output.create_dataset(
            "/state/gripper_cmd", data=np.asarray(gripper, dtype=np.uint8)
        )
        output.create_dataset("/time/stamp_rgb", data=stamp)
        output.create_dataset("/time/stamp_joint", data=state_stamp)
        output.create_dataset("/time/stamp_ee", data=state_stamp)
        output.create_dataset("/time/stamp_recv", data=stamp)
        output.attrs.update({
            "episode_uuid": path.stem,
            "record_hz": 20.0,
            "sync_slop_sec": 0.1 if jitter else 0.03,
            "data_block": "main",
            "position_id": position_id,
        })


def test_compare_batches_reports_stats_and_continues_after_candidate_qa_failure(
    tmp_path, capsys
):
    reference = tmp_path / "reference"
    candidate = tmp_path / "candidate"
    reference.mkdir()
    candidate.mkdir()
    write_episode(reference / "episode_r1.h5", [1, 2, 3])
    write_episode(reference / "episode_r2.h5", [3, 4, 5])
    write_episode(candidate / "episode_c1.h5", [4, 6, 8])
    write_episode(candidate / "episode_c2.h5", [6, 8, 10], jitter=True)
    write_episode(
        candidate / "episode_bad_close.h5",
        [100, 100, 100],
        gripper=[0, 1, 0, 1, 1, 1, 1],
    )

    result = compare_batches(reference, candidate)

    np.testing.assert_allclose(result["reference_mean_xyz_mm"], [2, 3, 4])
    np.testing.assert_allclose(result["candidate_mean_xyz_mm"], [5, 7, 9])
    np.testing.assert_allclose(
        result["candidate_minus_reference_xyz_mm"], [3, 4, 5]
    )
    assert np.isclose(result["l2_mm"], np.sqrt(50))
    assert result["candidate_qa_pass"] == 2
    assert result["candidate_qa_fail"] == 1
    assert len(result["candidate_records"]) == 2
    output = capsys.readouterr().out
    assert "file=episode_c2.h5 qa=FAIL" in output
    assert "file=episode_bad_close.h5" in output
    assert "expected exactly one 0->1 close transition, got 2" in output
    assert "DELTA candidate_minus_reference_xyz_mm=(3.000, 4.000, 5.000)" in output


def test_complete_h5_files_ignores_partial_files_and_position_mismatch(tmp_path, capsys):
    reference = tmp_path / "reference"
    candidate = tmp_path / "candidate"
    reference.mkdir()
    candidate.mkdir()
    write_episode(reference / "episode_ref.h5", [1, 1, 1])
    write_episode(candidate / "episode_good.h5", [2, 2, 2])
    write_episode(
        candidate / "episode_wrong_position.h5",
        [9, 9, 9],
        position_id="grid_4",
    )
    (candidate / "episode_unfinished.partial.h5").write_bytes(b"not an HDF5 file")

    assert [path.name for path in complete_h5_files(candidate)] == [
        "episode_good.h5",
        "episode_wrong_position.h5",
    ]
    result = compare_batches(reference, candidate)

    assert len(result["candidate_records"]) == 1
    np.testing.assert_allclose(result["candidate_mean_xyz_mm"], [2, 2, 2])
    output = capsys.readouterr().out
    assert "position_id='grid_4', expected 'grid_5'" in output
