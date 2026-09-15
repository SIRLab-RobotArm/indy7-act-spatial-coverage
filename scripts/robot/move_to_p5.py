#!/usr/bin/python3
"""Legacy 2026-08-14 P5 board-alignment helper, not an evaluation target.

The hard-coded pose comes from the older ``episodes/pre_recollection_episodes/p5`` diagnostic
batch.  Do not use it to populate ``experiment/positions.yaml`` or rollout
``p_target`` for the 2026-08-17 recollection experiment.
"""

import os

from neuromeka import IndyDCP3


ROBOT_IP = os.environ.get("GRIP_INDY_IP", "192.168.0.10")

HOME_JOINT_DEG = [
    0.005221,
    40.003174,
    -129.996640,
    90.000870,
    0.000435,
    0.001577,
]

# Mean close-EEF pose over the 52 valid successes in
# episodes/pre_recollection_episodes/p5
# [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
P5_MEAN_POSE = [
    606.107,
    5.479,
    539.644,
    89.989,
    -0.003,
    89.997,
]

# Close joint pose of the real successful demonstration nearest the P5 mean
# (episode_15, 1.313 mm away). HOME is wrist-singular, so we do not start a
# movel from HOME - move to this pose in joint space first.
P5_REFERENCE_JOINT_DEG = [
    14.781788,
    -5.660819,
    -92.645068,
    61.284241,
    16.923460,
    29.796335,
]

VELOCITY_RATIO = 5.0
ACCELERATION_RATIO = 20.0


def main():
    indy = IndyDCP3(robot_ip=ROBOT_IP, index=0)

    try:
        indy.stop_teleop()
        indy.stop_motion()

        print("Moving to HOME")
        indy.movej(
            jtarget=HOME_JOINT_DEG,
            vel_ratio=VELOCITY_RATIO,
            acc_ratio=ACCELERATION_RATIO,
        )
        indy.wait_for_motion_state("is_target_reached")

        print("Moving to the P5 reference joint pose")
        indy.movej(
            jtarget=P5_REFERENCE_JOINT_DEG,
            vel_ratio=VELOCITY_RATIO,
            acc_ratio=ACCELERATION_RATIO,
        )
        indy.wait_for_motion_state("is_target_reached")

        print("Correcting 1.313 mm onto the P5 mean position")
        indy.movel(
            ttarget=P5_MEAN_POSE,
            vel_ratio=VELOCITY_RATIO,
            acc_ratio=ACCELERATION_RATIO,
        )
        indy.wait_for_motion_state("is_target_reached")

        print("Arrived at the P5 mean position")

    except KeyboardInterrupt:
        indy.stop_motion()
        raise


if __name__ == "__main__":
    main()
