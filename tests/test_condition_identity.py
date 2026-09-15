from argparse import Namespace
import json

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from ai.convert.subset_dataset import build_subset


def test_rgb_dataset_has_three_channels_and_act_features(converted_dataset):
    root = converted_dataset["dataset"]
    repo_id = converted_dataset["repo_id"]
    dataset = LeRobotDataset(repo_id, root=root)
    key = "observation.images.overhead"
    sample = dataset[0]
    assert sample[key].shape[0] == 3
    assert sample["observation.state"].shape[-1] == 10
    assert sample["action"].shape[-1] == 4


def test_rgb_subset_selects_all_phases_for_raw_episode(converted_dataset, tmp_path):
    output = tmp_path / "subset"
    manifest = tmp_path / "subset_manifest.json"
    result = build_subset(Namespace(
        source_dir=str(converted_dataset["dataset"]),
        output_dir=str(output),
        manifest=str(manifest),
        episodes="",
        source_manifest=str(converted_dataset["manifest"]),
        episode_uuids="mock-0",
        source_repo_id=converted_dataset["repo_id"],
        output_repo_id="sirlab/test_grip_subset",
    ))

    subset = LeRobotDataset("sirlab/test_grip_subset", root=output)
    assert len(subset) == 3
    assert subset.num_episodes == 3
    assert subset[0]["observation.images.overhead"].shape[0] == 3
    assert result["selected_source_episode_uuids"] == ["mock-0"]
    assert json.loads(manifest.read_text())["frame_count"] == 3
