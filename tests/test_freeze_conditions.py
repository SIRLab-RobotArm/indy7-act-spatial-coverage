from ai.convert.freeze_conditions import CONDITION_C_COUNTS, freeze_membership


def synthetic_source_manifest():
    episodes = []
    derived = []
    derived_index = 0
    for position_index in range(1, 10):
        count = 60 if position_index == 5 else 20
        for episode_index in range(count):
            episode_uuid = f"p{position_index}-{episode_index:02d}"
            episodes.append({
                "episode_uuid": episode_uuid,
                "path": f"/episodes/p{position_index}/episode_{episode_uuid}.h5",
            })
            for phase in range(4):
                derived.append({
                    "dataset_episode_index": derived_index,
                    "episode_uuid": episode_uuid,
                    "phase": phase,
                })
                derived_index += 1
    return {"episodes": episodes, "derived_episodes": derived}


def test_frozen_condition_counts_and_nesting_are_deterministic():
    source = synthetic_source_manifest()
    first = freeze_membership(source, "selection-v1")
    second = freeze_membership(source, "selection-v1")

    assert first == second
    assert first["a"]["raw_episode_count"] == 60
    assert first["b"]["raw_episode_count"] == 60
    assert first["c"]["raw_episode_count"] == 60
    assert first["d"]["raw_episode_count"] == 180
    assert first["c"]["counts_by_position"] == CONDITION_C_COUNTS
    assert set(first["c"]["episode_uuids"]) < set(first["d"]["episode_uuids"])
    assert first["a"]["derived_episode_count"] == 240
    assert first["d"]["derived_episode_count"] == 720


def test_selection_seed_changes_sampled_membership():
    source = synthetic_source_manifest()
    first = freeze_membership(source, "selection-v1")
    second = freeze_membership(source, "selection-v2")

    assert first["c"]["membership_sha256"] != second["c"]["membership_sha256"]
    assert first["d"]["membership_sha256"] != second["d"]["membership_sha256"]
