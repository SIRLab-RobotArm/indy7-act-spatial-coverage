# Third-party components

The MIT license in `LICENSE` covers the code and documentation written for this
project. The components below came from elsewhere and keep their own terms.

## LeRobot 0.5.1 (vendored)

- Location: `ai/lerobot_source/lerobot/`
- Upstream: https://github.com/huggingface/lerobot, tag `v0.5.1`
- License: Apache License 2.0 (`ai/lerobot_source/lerobot/LICENSE`)

A pinned copy is vendored rather than declared as a dependency so that the exact
training and inference code used for the published results stays reproducible
even if upstream changes. `ai/lerobot_source/UPSTREAM.md` records the tag, the
tag object hash and an aggregate SHA-256 over the tree, so the copy can be
verified against upstream. No patches were applied (`ai/patches/` is empty).

## Neuromeka Indy7 driver (not included)

This repository contains **no** Neuromeka code. The robot-side packages depend
on `indy_interfaces`, `indy_driver` and `indy_description`, which you clone
separately into `ros2_ws/src/indy7_ros2/` (see the top-level README).

- Upstream: https://github.com/neuromeka-robotics/indy-ros2, branch
  `humble-indyDCP3` at `95a5a9c7465d50fd053704dfc5b0b8700783323d`
- Version used in the experiment: https://github.com/SIRLab-RobotArm/indy-ros2,
  tag `indy7-act-spatial-coverage-v1` (a GitHub fork of upstream; its
  `SIRLAB_CHANGES.md` lists the changes)
- License: upstream ships no license file. Its `package.xml` files declare
  `BSD-3-Clause`, except `indy_interfaces`, which declares none. The MIT license
  of this repository does not apply to it; check with Neuromeka before
  redistributing it.

## Mand.ro Mark7 hand description

- Location: `ros2_ws/src/mark7/pipet_hand_mark7_description/`
- License: see `ros2_ws/src/mark7/pipet_hand_mark7_description/LICENSE`

The URDF, meshes and original attribution for the Mark7 hand. The driver,
message and teleoperation packages next to it (`pipet_hand_mark7_driver`,
`pipet_hand_mark7_msgs`, `pipet_hand_mark7_teleop`) were written for this
project and are MIT-licensed like the rest of the repository.

**Open item:** the `LICENSE` in that package (Toshinori Kitamura, 2018) appears
to come from the URDF export tooling rather than from Mand.ro, so it may not
cover the hand geometry in `meshes/`. Redistribution of the meshes has not been
confirmed with Mand.ro.

## Mark7 serial protocol

`pipet_hand_mark7_driver` implements Mand.ro's serial protocol document ("Mand.ro
Mark 7 Communication Protocol", last updated 2026-03-05). The document itself is
not redistributed here; request it from Mand.ro.

## Recorded provenance paths

The JSON manifests under `experiment/manifests/` record absolute paths on the
machine where the data was acquired. They are left exactly as written, on
purpose: each condition manifest stores a `source_manifest_sha256` over the
manifest it was derived from, so editing those files would break the chain that
proves which raw episodes went into which training condition. The paths contain
no personal information.
