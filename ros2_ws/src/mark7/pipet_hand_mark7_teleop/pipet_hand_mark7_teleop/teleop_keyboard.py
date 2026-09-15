#!/usr/bin/env python3
"""
Mark7 command input node.
Usage: type six space-separated numbers, then Enter.
Example:  100 100 100 100 0 0
Joint order: Thumb (0-187)  Index  Middle  Ring  Pinky  ThAb  (0-300)
"""
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINT_DISPLAY = ['Thumb', 'Index', 'Middle', 'Ring ', 'Pinky', 'ThAb ']
JOINT_LIMITS  = [(0, 187), (0, 300), (0, 300), (0, 300), (0, 300), (0, 300)]

HELP_MSG = """\
=== Mark7 Command Input ===
Joint order: Thumb Index Middle Ring Pinky ThAb
Range: Thumb 0-187, the others 0-300

Example input) 100 100 100 100 0 0
         0 0 0 0 0 0   <- reset everything
Ctrl+C  quit
===========================
"""


class CommandInput(Node):
    def __init__(self):
        super().__init__('mark7_command_input')
        self._pub = self.create_publisher(
            Float64MultiArray,
            '/mark7/forward_position_controller/commands',
            10,
        )

    def send(self, values: list[float]):
        msg = Float64MultiArray()
        msg.data = values
        self._pub.publish(msg)
        status = '  '.join(
            f'{JOINT_DISPLAY[i]}:{v:.0f}' for i, v in enumerate(values)
        )
        print(f'  → {status}')


def parse_input(line: str):
    """Parse six space-separated numbers. Return None if that fails."""
    parts = line.strip().split()
    if len(parts) != 6:
        return None
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    # Clamp to the allowed range
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        values[i] = float(max(lo, min(hi, values[i])))
    return values


def main(args=None):
    rclpy.init(args=args)
    node = CommandInput()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(HELP_MSG)

    try:
        while rclpy.ok():
            try:
                line = input('> ')
            except EOFError:
                break

            values = parse_input(line)
            if values is None:
                print('  Error: enter six numbers separated by spaces')
                continue

            node.send(values)

    except KeyboardInterrupt:
        pass
    finally:
        print('\nexiting')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
