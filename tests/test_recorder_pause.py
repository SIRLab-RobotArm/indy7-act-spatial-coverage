import pytest

pytest.importorskip(
    "rclpy",
    reason="needs ROS 2 on the path: source /opt/ros/jazzy/setup.bash and "
           "ros2_ws/install/setup.bash")
pytest.importorskip(
    "grip_collect",
    reason="needs this repository's built colcon workspace: "
           "source ros2_ws/install/setup.bash")

from grip_collect.episode_recorder_node import EpisodeRecorderNode


class _Writer:
    count = 17

    def __init__(self):
        self.closed_attrs = None

    def close(self, attrs):
        self.closed_attrs = attrs
        return 'episode.h5'


def test_pause_latches_at_current_frame_and_blocks_sync_append():
    node = EpisodeRecorderNode.__new__(EpisodeRecorderNode)
    node.writer = _Writer()
    node.recording_paused = False
    node.raw_counts = {'rgb': 25, 'joint': 17, 'ee': 17}
    node.pause_raw_counts = None

    assert node.pause_episode() == 17
    assert node.recording_paused is True
    assert node.pause_raw_counts == {'rgb': 25, 'joint': 17, 'ee': 17}

    # Paused callbacks return before decoding or appending any message.
    node._sync_callback(None, None, None)


def test_pause_rejects_duplicate_request():
    node = EpisodeRecorderNode.__new__(EpisodeRecorderNode)
    node.writer = _Writer()
    node.recording_paused = True

    with pytest.raises(RuntimeError, match="already paused"):
        node.pause_episode()


def test_stop_uses_pause_boundary_for_sync_qa():
    writer = _Writer()
    node = EpisodeRecorderNode.__new__(EpisodeRecorderNode)
    node.writer = writer
    node.recording_paused = True
    node.pause_raw_counts = {'rgb': 25, 'joint': 17, 'ee': 17}
    # Simulate messages continuing during the operator's label choice.
    node.raw_counts = {'rgb': 70, 'joint': 47, 'ee': 47}
    node.sync_count = 17
    node.mock = False
    node.success_label = 'success'
    node.discard_reason = ''
    node.drop_conversion = 0
    node.auto_round_index = False

    assert node.stop_episode() == 'episode.h5'
    assert writer.closed_attrs['raw_topic_counts'] == {
        'rgb': 25, 'joint': 17, 'ee': 17,
    }
    assert writer.closed_attrs['dropped_sync_estimate'] == 0
