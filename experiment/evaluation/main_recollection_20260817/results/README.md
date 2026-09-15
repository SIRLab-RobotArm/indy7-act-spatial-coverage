# Evaluation results

The compact, publishable outputs of the 480-trial evaluation. These are the only
result files small enough to keep in git, and they are enough to verify every
number in the paper.

- `rollouts.csv` - one row per schedule row: the attempt that passed the
  validator, with grasp position, error and lift metrics
- `attempts.csv` - every attempt that was created, marked eligible, excluded or
  incomplete
- `summary.json` - completion, per-condition, per-model and per-position success
  rates, Wilson intervals and error summaries

The original `trace.parquet`, `events.json` and ROS bags stay in
`experiment/rollouts/main_recollection_20260817_main_smooth30hz`, out of git
because of their size.

## Regenerating these files

This rewrites only this directory; it never modifies the original traces. It
needs the raw rollouts, so it only works on a machine that has them.

```bash
source scripts/env.sh && grip_activate_conda
python scripts/eval/export_results.py --require-complete
```

While `provisional` is `true` in `summary.json`, the numbers are an interim
snapshot. After all 480 trials it is regenerated with `--require-complete`, and
the git commit and SHA-256 of the analysis artifacts are recorded in the paper.

## Final summary

Regenerated and verified with `--require-complete` on 2026-09-01. `provisional`
is `false`. Each condition was evaluated 120 times.

| Condition | Successes / trials | Success rate | Wilson 95% CI |
| --- | ---: | ---: | ---: |
| A | 15 / 120 | 12.5% | 7.7-19.6% |
| B | 74 / 120 | 61.7% | 52.7-69.9% |
| C | 96 / 120 | 80.0% | 72.0-86.2% |
| D | 116 / 120 | 96.7% | 91.7-98.7% |
| Overall | 301 / 480 | 62.7% | - |

Split by evaluation group:

| Condition | Exact grid | Q, never demonstrated |
| --- | ---: | ---: |
| A | 15 / 60 (25.0%) | 0 / 60 (0.0%) |
| B | 47 / 60 (78.3%) | 27 / 60 (45.0%) |
| C | 52 / 60 (86.7%) | 44 / 60 (73.3%) |
| D | 58 / 60 (96.7%) | 58 / 60 (96.7%) |

Of 510 attempt directories, 480 valid results matching a schedule row were
counted. One attempt was excluded, 29 were incomplete or invalid, and 5 outcome
corrections are recorded. The median 3-D close error over successful rollouts is
10.43 mm (n = 301). Per-model and per-position breakdowns, error distributions
and the raw Wilson intervals are in `summary.json`.

## Known limitation of the inferential model

The descriptive results above - the success rates, the Wilson intervals, the
grasp errors - are deterministic counting over `rollouts.csv`. Rerun
`python -m ai.eval.analyze --metrics-only` as often as you like and you get
byte-identical output.

The **inferential** part is different. `ai/eval/analyze.py` fits a Bayesian mixed
GLM with `statsmodels`' `fit_vb()`, and on this dataset that fit does not
converge. It emits `ConvergenceWarning`, and repeated runs land on different
posterior means - sometimes collapsing to all zeros. The mixed linear models for
E1/E2/E3 warn similarly.

So the model coefficients from a full `analyze` run are **not reproducible run to
run**, and should not be quoted as stable estimates without first addressing the
convergence problem. Nothing in the headline table depends on them.
