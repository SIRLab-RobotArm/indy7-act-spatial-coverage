import h5py
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from ai.eval.action_contract import executor_command


def test_hdf5_dataset_executor_action_roundtrip(converted_dataset):
    raw_path = converted_dataset["episodes"] / "episode_0.h5"
    with h5py.File(raw_path, "r") as episode:
        expected_xyz = (
            np.asarray(episode["/state/ee_pose"][4, :3], dtype=np.float32)
            - np.asarray(episode["/state/ee_pose"][0, :3], dtype=np.float32)
        )
        expected_gripper = int(episode["/state/gripper_cmd"][4])
    dataset = LeRobotDataset(converted_dataset["repo_id"], root=converted_dataset["dataset"])
    action = dataset[0]["action"].numpy()
    np.testing.assert_array_equal(action[:3], expected_xyz)
    assert int(action[3]) == expected_gripper
    pose_command, gripper_command = executor_command(action)
    np.testing.assert_array_equal(pose_command[:3], expected_xyz)
    np.testing.assert_array_equal(pose_command[3:], np.zeros(3, dtype=np.float32))
    assert gripper_command == expected_gripper
