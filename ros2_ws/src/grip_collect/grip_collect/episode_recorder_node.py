#!/usr/bin/env python3
"""Synchronized incremental HDF5 recorder for the ICRiTA experiment."""

import argparse
from contextlib import suppress
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

import h5py
import message_filters
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterEvent
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from indy_interfaces.msg import EefPose


SCHEMA_VERSION = 2
RECORD_HZ = 20.0
# D435 color exposure uses UVC exposure-absolute units of 100 microseconds.
REALSENSE_EXPOSURE_UNIT_US = 100.0
CAMERA_PARAMETER_NAMES = (
    'rgb_camera.exposure',
    'rgb_camera.gain',
    'rgb_camera.white_balance',
    'rgb_camera.enable_auto_exposure',
    'rgb_camera.enable_auto_white_balance',
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_to_numpy(msg, kind):
    """Decode ROS Image data while respecting row stride and byte order."""
    if kind == 'rgb':
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise ValueError(f'unsupported RGB encoding {msg.encoding!r}')
        row_bytes = msg.width * 3
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = rows[:, :row_bytes].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            image = image[..., ::-1]
        return np.ascontiguousarray(image)
    raise ValueError(f'unknown image kind {kind!r}')


def repository_sha():
    root = Path(__file__).resolve().parents[4]
    try:
        return subprocess.check_output(
            ['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


class HDF5EpisodeWriter:
    """Append-only RGB episode writer; raw action is deliberately absent."""

    def __init__(self, output_dir, metadata, image_shape=(480, 640)):
        output_dir = Path(output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        self.episode_uuid = str(uuid.uuid4())
        self.partial_path = output_dir / f'episode_{self.episode_uuid}.partial.h5'
        self.final_path = output_dir / f'episode_{self.episode_uuid}.h5'
        self.file = h5py.File(self.partial_path, 'w', libver='latest')
        self.count = 0
        height, width = image_shape
        self.datasets = {
            'rgb': self.file.create_dataset(
                '/obs/rgb', shape=(0, height, width, 3),
                maxshape=(None, height, width, 3), dtype='u1',
                chunks=(1, height, width, 3), compression='lzf'),
            'ee_pose': self.file.create_dataset(
                '/state/ee_pose', shape=(0, 6), maxshape=(None, 6),
                dtype='<f4', chunks=(256, 6)),
            'joint_pos': self.file.create_dataset(
                '/state/joint_pos', shape=(0, 6), maxshape=(None, 6),
                dtype='<f4', chunks=(256, 6)),
            'gripper_cmd': self.file.create_dataset(
                '/state/gripper_cmd', shape=(0,), maxshape=(None,),
                dtype='u1', chunks=(1024,)),
            'stamp_rgb': self.file.create_dataset(
                '/time/stamp_rgb', shape=(0,), maxshape=(None,), dtype='<f8', chunks=(1024,)),
            'stamp_joint': self.file.create_dataset(
                '/time/stamp_joint', shape=(0,), maxshape=(None,), dtype='<f8', chunks=(1024,)),
            'stamp_ee': self.file.create_dataset(
                '/time/stamp_ee', shape=(0,), maxshape=(None,), dtype='<f8', chunks=(1024,)),
            'stamp_recv': self.file.create_dataset(
                '/time/stamp_recv', shape=(0,), maxshape=(None,), dtype='<f8', chunks=(1024,)),
        }
        self.file.attrs['schema_version'] = SCHEMA_VERSION
        self.file.attrs['episode_uuid'] = self.episode_uuid
        self.file.attrs['created_utc'] = datetime.now(timezone.utc).isoformat()
        for key, value in metadata.items():
            self.set_attr(key, value)
        self.file.flush()

    def set_attr(self, key, value):
        if isinstance(value, (dict, list, tuple)) and not (
                isinstance(value, (list, tuple)) and all(
                    isinstance(item, (int, float, np.number)) for item in value)):
            value = json.dumps(value, sort_keys=True)
        self.file.attrs[key] = value

    def append(self, *, rgb, ee_pose, joint_pos, gripper_cmd,
               stamp_rgb, stamp_joint, stamp_ee, stamp_recv):
        values = {
            'rgb': rgb,
            'ee_pose': ee_pose,
            'joint_pos': joint_pos,
            'gripper_cmd': gripper_cmd,
            'stamp_rgb': stamp_rgb,
            'stamp_joint': stamp_joint,
            'stamp_ee': stamp_ee,
            'stamp_recv': stamp_recv,
        }
        index = self.count
        for key, dataset in self.datasets.items():
            dataset.resize(index + 1, axis=0)
            dataset[index] = values[key]
        self.count += 1
        self.file.flush()

    def close(self, extra_attrs=None):
        if self.file is None:
            return self.final_path
        self.file.attrs['frame_count'] = self.count
        for key, value in (extra_attrs or {}).items():
            self.set_attr(key, value)
        self.file.flush()
        self.file.close()
        self.file = None
        self.partial_path.replace(self.final_path)
        return self.final_path

    def discard(self, reason):
        if self.file is not None:
            self.file.attrs['discard_reason'] = str(reason)
            self.file.flush()
            self.file.close()
            self.file = None
        self.partial_path.unlink(missing_ok=True)


class EpisodeRecorderNode(Node):
    def __init__(self, force_mock=False, cli_output_dir=None):
        super().__init__('episode_recorder')
        self.declare_parameter('mock', bool(force_mock))
        self.declare_parameter('output_dir', cli_output_dir or 'episodes/development_episodes')
        self.declare_parameter('data_block', 'dev')
        self.declare_parameter('sync_slop', 0.03)
        self.declare_parameter('session_id', 'dev_session')
        self.declare_parameter('operator_id', 'dev_operator')
        self.declare_parameter('position_id', 'pilot_1')
        self.declare_parameter('round_index', -1)
        self.declare_parameter('accept_context_updates', True)
        self.declare_parameter('camera_serial', 'UNKNOWN')
        self.declare_parameter('exposure_us', 5000.0)
        self.declare_parameter('gain', 16.0)
        self.declare_parameter('white_balance', 4600.0)
        self.declare_parameter('auto_exposure_locked', True)
        self.declare_parameter('approach_orientation_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('home_joint_deg', [0.0, 25.0, -115.0, 90.0, 0.0, 0.0])
        self.declare_parameter('tcp_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('ref_frame', 'indy_base')
        self.declare_parameter('image_height', 480)
        self.declare_parameter('image_width', 640)

        self.mock = bool(force_mock or self.get_parameter('mock').value)
        self.output_dir = Path(str(self.get_parameter('output_dir').value)).resolve()
        self.data_block = str(self.get_parameter('data_block').value)
        if self.data_block not in ('dev', 'pilot', 'main'):
            raise ValueError('data_block must be dev, pilot, or main')
        if self.data_block == 'dev' and '_dev' not in self.output_dir.parts:
            raise ValueError('dev episodes must be stored under an _dev directory')
        self.image_shape = (
            int(self.get_parameter('image_height').value),
            int(self.get_parameter('image_width').value))
        self.writer = None
        self.recording_paused = False
        self.pause_raw_counts = None
        self.gripper_cmd = 0
        self.success_label = 'unlabeled'
        self.discard_reason = ''
        self.drop_conversion = 0
        self.sync_count = 0
        self.raw_counts = {'rgb': 0, 'joint': 0, 'ee': 0}
        self.color_intrinsics = {}
        self.camera_readback = {}
        self.camera_readback_pending = False
        self.position_id = str(self.get_parameter('position_id').value)
        if self.position_id not in {
                *(f'grid_{index}' for index in range(1, 10)),
                *(f'eval_{index}' for index in range(1, 9)),
                'pilot_1', 'pilot_2'}:
            raise ValueError(f'invalid position_id: {self.position_id}')
        configured_round = int(self.get_parameter('round_index').value)
        self.auto_round_index = configured_round < 0
        self.round_index = (
            len(list(self.output_dir.glob('episode_*.h5')))
            if self.auto_round_index else configured_round)
        self.accept_context_updates = bool(
            self.get_parameter('accept_context_updates').value)

        self.create_service(Trigger, '/grip_collect/start', self._start_callback)
        self.create_service(Trigger, '/grip_collect/pause', self._pause_callback)
        self.create_service(Trigger, '/grip_collect/stop', self._stop_callback)
        self.create_service(Trigger, '/grip_collect/discard', self._discard_callback)
        self.create_service(Trigger, '/grip_collect/mark_success', self._success_callback)
        self.create_service(Trigger, '/grip_collect/mark_fail', self._fail_callback)
        self.create_service(Trigger, '/grip_collect/gripper/open', self._open_callback)
        self.create_service(Trigger, '/grip_collect/gripper/close', self._close_callback)
        self.recording_pub = self.create_publisher(Bool, '/grip_collect/is_recording', 1)
        self.status_pub = self.create_publisher(String, '/grip_collect/status', 10)
        self.create_timer(0.5, self._publish_status)
        self.create_subscription(String, '/grip_collect/context', self._context_callback, 10)

        if not self.mock:
            self._setup_subscriptions(float(self.get_parameter('sync_slop').value))
            self.camera_param_client = AsyncParameterClient(
                self, '/overhead_camera/camera')
            self.camera_param_timer = self.create_timer(1.0, self._query_camera_parameters)
        self.get_logger().info(
            f'EpisodeRecorder initialized (mock={self.mock}, output={self.output_dir})')

    def _setup_subscriptions(self, slop):
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/overhead_camera/camera/color/image_raw')
        self.joint_sub = message_filters.Subscriber(self, JointState, '/joint_states')
        self.ee_sub = message_filters.Subscriber(self, EefPose, '/indy/ee_pose_stamped')
        for name, subscriber in (
                ('rgb', self.rgb_sub), ('joint', self.joint_sub),
                ('ee', self.ee_sub)):
            subscriber.registerCallback(lambda _msg, key=name: self._count_raw(key))
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.joint_sub, self.ee_sub],
            queue_size=20, slop=slop)
        self.sync.registerCallback(self._sync_callback)
        self.create_subscription(
            CameraInfo, '/overhead_camera/camera/color/camera_info',
            lambda msg: self._camera_info(msg, 'color'), 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self._parameter_event, 10)

    def _count_raw(self, key):
        self.raw_counts[key] += 1

    def _camera_info(self, msg, kind):
        value = {
            'width': int(msg.width), 'height': int(msg.height),
            'distortion_model': msg.distortion_model,
            'd': list(msg.d), 'k': list(msg.k), 'r': list(msg.r), 'p': list(msg.p),
        }
        self.color_intrinsics = value
        if self.writer is not None:
            self.writer.set_attr(f'{kind}_intrinsics', value)

    def _parameter_event(self, event):
        if 'overhead_camera' not in event.node:
            return
        for parameter in [*event.new_parameters, *event.changed_parameters]:
            if parameter.name in CAMERA_PARAMETER_NAMES:
                self.camera_readback[parameter.name] = parameter_value_to_python(
                    parameter.value)
        if self.writer is not None:
            self.writer.set_attr('camera_parameter_readback', self.camera_readback)
            self.writer.set_attr(
                'camera_readback_verified', self._camera_readback_verified())

    def _camera_readback_verified(self):
        expected = {
            'rgb_camera.exposure': (
                float(self.get_parameter('exposure_us').value)
                / REALSENSE_EXPOSURE_UNIT_US
            ),
            'rgb_camera.gain': float(self.get_parameter('gain').value),
            'rgb_camera.white_balance': float(self.get_parameter('white_balance').value),
            'rgb_camera.enable_auto_exposure': False,
            'rgb_camera.enable_auto_white_balance': False,
        }
        for name, expected_value in expected.items():
            actual_value = self.camera_readback.get(name)
            if actual_value is None:
                return False
            if isinstance(expected_value, bool):
                if bool(actual_value) is not expected_value:
                    return False
            elif not np.isclose(
                    float(actual_value), expected_value, rtol=0.0, atol=1e-6):
                return False
        return True

    def _query_camera_parameters(self):
        if self.camera_readback_pending or self._camera_readback_verified():
            return
        if not self.camera_param_client.services_are_ready():
            return
        self.camera_readback_pending = True
        future = self.camera_param_client.get_parameters(list(CAMERA_PARAMETER_NAMES))
        future.add_done_callback(self._camera_parameters_done)

    def _camera_parameters_done(self, future):
        self.camera_readback_pending = False
        try:
            response = future.result()
            for name, value in zip(CAMERA_PARAMETER_NAMES, response.values):
                self.camera_readback[name] = parameter_value_to_python(value)
        except Exception as exc:
            self.get_logger().error(f'Camera parameter readback failed: {exc}')
            return
        verified = self._camera_readback_verified()
        self.get_logger().info(f'Camera parameter readback verified={verified}')
        if self.writer is not None:
            self.writer.set_attr('camera_parameter_readback', self.camera_readback)
            self.writer.set_attr('camera_readback_verified', verified)

    def _metadata(self):
        keys = [
            'session_id', 'operator_id', 'position_id', 'round_index', 'camera_serial',
            'exposure_us', 'gain', 'white_balance', 'auto_exposure_locked',
            'approach_orientation_deg', 'home_joint_deg', 'tcp_offset',
            'ref_frame']
        metadata = {key: self.get_parameter(key).value for key in keys}
        metadata['position_id'] = self.position_id
        metadata['round_index'] = self.round_index
        metadata.update({
            'data_block': self.data_block,
            'observation_mode': 'rgb_only',
            'git_sha': repository_sha(),
            'success_label': self.success_label,
            'discard_reason': self.discard_reason,
            'color_intrinsics': self.color_intrinsics,
            'camera_parameter_readback': self.camera_readback,
            'camera_readback_verified': self._camera_readback_verified(),
            'sync_slop_sec': float(self.get_parameter('sync_slop').value),
            'record_hz': RECORD_HZ,
            'raw_action_stored': False,
            'camera_exposure_unit_us': REALSENSE_EXPOSURE_UNIT_US,
            'camera_exposure_raw_expected': (
                float(self.get_parameter('exposure_us').value)
                / REALSENSE_EXPOSURE_UNIT_US
            ),
        })
        return metadata

    def _context_callback(self, msg):
        if not self.accept_context_updates:
            return
        if self.writer is not None:
            self.get_logger().warning('Ignoring collection context change while recording')
            return
        try:
            context = json.loads(msg.data)
            position_id = str(context['position_id'])
            round_index = int(context['round_index'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'Invalid /grip_collect/context: {exc}')
            return
        if position_id not in {
                *(f'grid_{index}' for index in range(1, 10)),
                'center_extra', 'pilot_1', 'pilot_2'}:
            self.get_logger().error(f'Invalid position_id: {position_id}')
            return
        self.position_id = position_id
        self.round_index = round_index

    def start_episode(self):
        if self.writer is not None:
            raise RuntimeError('already recording')
        self.success_label = 'unlabeled'
        self.discard_reason = ''
        self.drop_conversion = 0
        self.sync_count = 0
        self.raw_counts = {key: 0 for key in self.raw_counts}
        self.recording_paused = False
        self.pause_raw_counts = None
        self.writer = HDF5EpisodeWriter(self.output_dir, self._metadata(), self.image_shape)
        return self.writer.partial_path

    def pause_episode(self):
        if self.writer is None:
            raise RuntimeError('not recording')
        if self.recording_paused:
            raise RuntimeError('recording already paused')
        # Raw subscriptions continue while the operator chooses a label. Keep
        # the QA boundary identical to the frame boundary so label-wait
        # messages are not reported as synchronization drops.
        self.pause_raw_counts = dict(self.raw_counts)
        self.recording_paused = True
        return self.writer.count

    def stop_episode(self):
        if self.writer is None:
            raise RuntimeError('not recording')
        episode_raw_counts = (
            dict(self.pause_raw_counts)
            if self.pause_raw_counts is not None
            else dict(self.raw_counts)
        )
        drops = max(
            0, min(episode_raw_counts.values()) - self.sync_count
        ) if not self.mock else 0
        writer = self.writer
        self.writer = None
        self.recording_paused = False
        self.pause_raw_counts = None
        path = writer.close({
            'success_label': self.success_label,
            'discard_reason': self.discard_reason,
            'dropped_conversion': self.drop_conversion,
            'dropped_sync_estimate': drops,
            'raw_topic_counts': episode_raw_counts,
        })
        if self.auto_round_index:
            self.round_index += 1
        return path

    def _start_callback(self, _request, response):
        try:
            path = self.start_episode()
            response.success = True
            response.message = f'recording: {path}'
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _stop_callback(self, _request, response):
        try:
            path = self.stop_episode()
            response.success = True
            response.message = f'saved: {path}'
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _pause_callback(self, _request, response):
        try:
            frame_count = self.pause_episode()
            response.success = True
            response.message = f'paused after {frame_count} frames'
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _discard_callback(self, _request, response):
        if self.writer is None:
            response.success = False
            response.message = 'not recording'
            return response
        self.discard_reason = 'operator_discard'
        self.writer.discard(self.discard_reason)
        self.writer = None
        self.recording_paused = False
        self.pause_raw_counts = None
        response.success = True
        response.message = 'discarded current partial episode'
        return response

    def _success_callback(self, _request, response):
        self.success_label = 'success'
        response.success = True
        response.message = 'success label latched'
        return response

    def _fail_callback(self, _request, response):
        self.success_label = 'fail'
        response.success = True
        response.message = 'failure label latched'
        return response

    def _open_callback(self, _request, response):
        self.gripper_cmd = 0
        response.success = True
        response.message = 'binary gripper OPEN latched'
        return response

    def _close_callback(self, _request, response):
        self.gripper_cmd = 1
        response.success = True
        response.message = 'binary gripper CLOSED latched'
        return response

    def _sync_callback(self, rgb_msg, joint_msg, ee_msg):
        if self.writer is None or self.recording_paused:
            return
        try:
            rgb = image_to_numpy(rgb_msg, 'rgb')
            if rgb.shape[:2] != self.image_shape:
                raise ValueError(
                    f'image shape mismatch: rgb={rgb.shape}, '
                    f'expected={self.image_shape}')
            joint_pos = list(joint_msg.position[:6])
            if len(joint_pos) != 6:
                raise ValueError(f'expected 6 joints, got {len(joint_pos)}')
        except Exception as exc:
            self.drop_conversion += 1
            self.get_logger().error(
                f'Dropped synchronized sample #{self.drop_conversion}: {exc}')
            return
        recv = self.get_clock().now().nanoseconds * 1e-9
        self.writer.append(
            rgb=rgb, ee_pose=list(ee_msg.pose), joint_pos=joint_pos,
            gripper_cmd=self.gripper_cmd,
            stamp_rgb=stamp_seconds(rgb_msg.header.stamp),
            stamp_joint=stamp_seconds(joint_msg.header.stamp),
            stamp_ee=stamp_seconds(ee_msg.header.stamp), stamp_recv=recv)
        self.sync_count += 1

    def append_mock_frame(self, index, base_stamp=None):
        if self.writer is None:
            raise RuntimeError('mock episode not started')
        height, width = self.image_shape
        base_stamp = time.time() if base_stamp is None else base_stamp
        stamp = base_stamp + index / RECORD_HZ
        x = np.arange(width, dtype=np.uint16)[None, :]
        y = np.arange(height, dtype=np.uint16)[:, None]
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[..., 0] = (x + index) % 256
        rgb[..., 1] = (y + 2 * index) % 256
        rgb[..., 2] = ((x + y) // 2 + 3 * index) % 256
        self.writer.append(
            rgb=rgb,
            ee_pose=[0.5 * index, -0.25 * index, 100.0 - 0.1 * index, 0, 0, 0],
            joint_pos=[0.001 * index] * 6,
            gripper_cmd=int(index >= 6),
            stamp_rgb=stamp,
            stamp_joint=stamp + 0.002, stamp_ee=stamp + 0.002,
            stamp_recv=stamp + 0.003)
        self.sync_count += 1

    def _publish_status(self):
        recording = Bool()
        recording.data = self.writer is not None
        self.recording_pub.publish(recording)
        status = String()
        status.data = json.dumps({
            'recording': recording.data,
            'frames': 0 if self.writer is None else self.writer.count,
            'sync_count': self.sync_count,
            'raw_counts': self.raw_counts,
            'dropped_conversion': self.drop_conversion,
            'data_block': self.data_block,
        }, sort_keys=True)
        self.status_pub.publish(status)


def _parse_args(args):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--output-dir', default='episodes/development_episodes')
    return parser.parse_known_args(args)


def main(args=None):
    parsed, ros_args = _parse_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = EpisodeRecorderNode(parsed.mock, parsed.output_dir)
    try:
        if parsed.mock:
            node.start_episode()
            base_stamp = time.time()
            for index in range(parsed.frames):
                node.append_mock_frame(index, base_stamp)
            print(node.stop_episode())
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer is not None:
            node.writer.discard('node_shutdown_before_stop')
            node.writer = None
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == '__main__':
    main()
