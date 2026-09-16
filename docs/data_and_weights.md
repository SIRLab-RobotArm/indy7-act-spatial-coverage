# Data, model weights and raw traces

Small, reproduction-critical files live in git. Large artifacts do not, because
they would make the repository unusable. `.gitignore` here is a routing
decision, not a statement that something is disposable.

## What is in this repository

- The full conversion, training, inference and evaluation code
- `experiment/positions.yaml` - the target pose of every position
- The frozen per-condition UUID and dataset manifests
- The 480-row evaluation schedule, the blind model key, and every protocol
  amendment
- The compact result tables under
  `experiment/evaluation/main_recollection_20260817/results/`

No raw images, HDF5 recordings, checkpoints or ROS bags are committed.

## What is not, and where it is

| Artifact | Size | Where it is now |
| --- | ---: | --- |
| Combined dataset, video encoded | 0.22 GB | [Hugging Face, CC BY 4.0](https://huggingface.co/datasets/SIRLab-HGU/indy7-act-spatial-coverage) |
| The 12 trained models, 100k checkpoints | 2.4 GB | [Hugging Face, Apache 2.0](https://huggingface.co/SIRLab-HGU/indy7-act-spatial-coverage-models) |
| Combined LeRobot dataset (`rgb_all`), PNG frames | 15 GB | Local; the video release stands in for it |
| Per-condition datasets A-D | 24 GB | Local, regenerable from `rgb_all` |
| Raw HDF5 demonstrations, 220 episodes | 90 GB | Local and lab storage |
| Raw rollout traces and ROS bags | 640 GB | Local and lab storage |

**Release status:** published on 2026-09-16. The raw HDF5 recordings were left
out: 90 GB is a poor fit for a free Hub account, and HDF5 is not one of the
formats the Hub reads. Request them by mail.

### What was published, and how it differs

The released dataset is `rgb_all` re-encoded as AV1 video with LeRobot's own
`convert_image_to_video_dataset` (CRF 30, GOP 2, `yuv420p`), not a fresh
conversion from HDF5. That choice keeps the frozen condition manifests valid,
because episode indices and lengths are preserved. States, actions, timestamps
and indices were verified identical frame by frame; pixels differ (max 93, mean
1.83, PSNR 40.4 dB over 500 sampled frames), partly because RGB stored as
`yuv420p` is lossy even with a lossless codec.

All 176,837 frames were scanned for skin tones before publishing: the highest
per-frame share was 0.086%, and a visual check of the worst 24 frames plus 46
spread across the dataset showed only the arm, the gripper and the pipe.

The weights keep the `train_config.json` they were trained with, including the
absolute paths of the training machine. They record no credentials (Weights &
Biases is disabled), and the paths are left in place as provenance rather than
rewritten to the Hub id.

Everything needed to verify the published numbers is already in the repository -
the raw artifacts are needed only to retrain from scratch or to re-derive the
result tables from the original traces.

## What a dataset release should contain

The natural public artifact is the combined LeRobot dataset
(`main_recollection_20260817_rgb_all`) plus the per-condition membership
manifests. The A-D datasets are exactly reproducible from those two with the
conversion code in this repository, so publishing them separately duplicates 24 GB
for no gain - unless bit-identical ready-to-train copies are specifically wanted.

A dataset card should state:

- The Indy7, Mark7, D435 and PVC pipe specifications
- RGB-only, 20 Hz observations, and the four-phase 5 Hz trajectory definition
- The 10-dimensional state and 4-dimensional action definitions
- 220 raw episodes and their per-position counts
- The membership SHA-256 of each condition A-D
- The selection rule: successful demonstrations only
- The limitation that this is one object, one lighting setup, one camera pose
- License, citation, and the code commit it corresponds to

The roughly 90 GB of raw HDF5 is better published as a separate raw dataset, or
behind a gated repository. Before publishing, strip `operator_id`, the camera
serial number and local absolute paths from the released copy; do not alter
UUIDs, sensor timestamps or any numeric data.

## What a model release should contain

Only the 12 final 100k checkpoints used in the paper:

```text
condition A/B/C/D x seed 0/1/2
└── checkpoints/last/pretrained_model
    ├── config.json
    ├── model.safetensors
    ├── policy_preprocessor.json
    ├── policy_preprocessor_step_3_normalizer_processor.safetensors
    ├── policy_postprocessor.json
    └── policy_postprocessor_step_0_unnormalizer_processor.safetensors
```

Note that each checkpoint's `train_config.json` contains the absolute dataset
path from the training machine. Replace it with the Hub dataset ID in the
released copy. The intermediate 20k-80k checkpoints were not used in any analysis
and do not belong in the default release; publish them as a separate archival
revision if someone needs them.

A model card should record the condition, the seed, the 100k training
configuration, the dataset it links to, the 480-trial evaluation result, the fact
that these are real-robot-only policies, and the hardware needed to run them
safely.

## Evaluation results

These are published because reanalysis depends on them, and they are small:

- 480 per-rollout rows that passed the validator
- The success/failure record and the outcome-correction audit trail
- E1/E2/E3 grasp errors, lift height and hold time
- Per-condition, per-model and per-position summaries, plus the analysis code
- The frozen schedule and every protocol amendment

The 640 GB of ROS bags stays out of the default release. It is preserved on lab
storage with checksums; representative success and failure videos, or the minimum
trace needed for a specific claim, can be published on request.

## Before publishing anything

1. [done] All 480 trials complete and `export_results.py --require-complete`
   passes
2. Manually review the outcome corrections and the incomplete attempts
3. [done] Release copies checked: no people in frame, no credentials in the
   weights; the training machine's absolute paths are kept on purpose
4. [done] Code MIT, dataset CC BY 4.0, weights Apache 2.0
5. Decide the public/private timing against the venue's anonymity policy - the
   code, data and weights are already public under the authors' real names
6. Record the SHA-256 values, the git commit, and the LeRobot and PyTorch
   versions
7. Smoke-test loading the dataset and all 12 checkpoints in a fresh environment
8. Review the dataset card, model card and citation text before going public
