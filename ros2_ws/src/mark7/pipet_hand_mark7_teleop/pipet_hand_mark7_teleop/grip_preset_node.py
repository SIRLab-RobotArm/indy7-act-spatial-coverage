#!/usr/bin/env python3
"""
Grip Preset Node for Mark7.

Provides /gripper/grasp, /gripper/open, /gripper/press services.
On service call, publishes preset joint values to
/mark7/forward_position_controller/commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class GripPresetNode(Node):

    def __init__(self):
        super().__init__('grip_preset_node')

        # Preset parameters (Thumb, Index, Middle, Ring, Pinky, ThumbAb)
        self.declare_parameter('grasp', [0.0, 350.0, 350.0, 350.0, 350.0, 0.0])
        self.declare_parameter('open', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('press', [150.0, 0.0, 350.0, 350.0, 350.0, 0.0])
        self.declare_parameter('release', [0.0, 0.0, 350.0, 350.0, 350.0, 0.0])

        # Publisher to forward_position_controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/mark7/forward_position_controller/commands',
            10,
        )

        # Services
        self.create_service(Trigger, '/gripper/grasp', self._grasp_cb)
        self.create_service(Trigger, '/gripper/open', self._open_cb)
        self.create_service(Trigger, '/gripper/press', self._press_cb)
        self.create_service(Trigger, '/gripper/release', self._release_cb)

        self.get_logger().info('Grip preset node ready')

    def _publish_preset(self, preset_name: str) -> str:
        values = self.get_parameter(preset_name).get_parameter_value().double_array_value
        msg = Float64MultiArray()
        msg.data = list(values)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'{preset_name}: {list(values)}')
        return f'{preset_name} command sent: {list(values)}'

    def _grasp_cb(self, request, response):
        response.message = self._publish_preset('grasp')
        response.success = True
        return response

    def _open_cb(self, request, response):
        response.message = self._publish_preset('open')
        response.success = True
        return response

    def _press_cb(self, request, response):
        response.message = self._publish_preset('press')
        response.success = True
        return response

    def _release_cb(self, request, response):
        response.message = self._publish_preset('release')
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripPresetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
