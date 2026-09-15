# Experiment records

The frozen design artifacts, training records and real-robot results of the
`main_recollection_20260817` experiment. Small reproduction metadata is committed
to git; large rollout traces and logs are not.

## Contents

| Path | Contents | In git |
| --- | --- | --- |
| `positions.yaml` | Target coordinates for G1-G9, Q1-Q4 and the evaluation points | yes |
| `positions.before_teach_20260809.yaml` | Snapshot of the same file before targets were finalised | yes |
| `preregistration.md` | The analysis plan, including when it was frozen and how reality deviated | yes |
| `manifests/` | Frozen UUID membership and dataset records for the full data and conditions A-D | yes |
| `evaluation/main_recollection_20260817/` | The 480-row schedule, the blind model key, protocol amendments, and the compact results | yes |
| `rollouts/main_recollection_20260817_main_smooth30hz/` | Per-trial traces, events, metadata and ROS bags for all 480 trials and their retries | no, ~640 GB |
| `training_logs/` | Training console logs for A-D x seeds 0/1/2 | no |

## The final evaluation

12 models x 8 positions x 5 repeats = 480 trials, all completed. The compact
files used for analysis:

```text
evaluation/main_recollection_20260817/results/
├── rollouts.csv       one row per schedule row: the attempt that passed the validator
├── attempts.csv       the full audit trail, including retries, exclusions and incomplete runs
├── summary.json       per-condition, per-model and per-position statistics with Wilson intervals
└── README.md          the human-readable result summary
```

Never commit or casually duplicate `rollouts/` - it is enormous. When an outcome
needs correcting, do not delete the original attempt; leave an
`outcome_correction` audit trail. Reproducing the results requires the schedule,
the model key, the protocol amendments and the original per-trial metadata
together.

## About the recorded paths

The JSON files under `manifests/` record absolute paths on the machine where the
data was acquired. They are left exactly as written. Each condition manifest
carries a `source_manifest_sha256` over the manifest it derives from, so editing
those files would break the chain that proves which raw episodes went into which
training condition. See [`../NOTICE.md`](../NOTICE.md).

## Directory rename record

On 2026-09-01 the episode directories were renamed - names only - to make their
roles clearer:

```text
main_recollection_20260817 → main_20260817
main                       → pre_recollection_episodes
_diagnostic                → diagnostic_episodes
_dev                       → development_episodes
```

HDF5 contents and UUIDs were untouched. Manifests and code references containing
absolute paths were updated, which changed the SHA-256 of
`main_recollection_20260817_rgb_all.json` and the `source_manifest_sha256` values
that point at it. Dataset content, membership and the source HDF5 checksums did
not change.
