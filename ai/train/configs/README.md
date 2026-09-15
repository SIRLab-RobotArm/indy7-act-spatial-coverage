# Training configuration matrix

The experiment has 12 RGB-only models: `condition={a,b,c,d}` crossed with
`seed={0,1,2}`.

> These YAML files are descriptive notes, not the configuration that was
> executed. The authoritative training configuration is
> `scripts/train/run_main_matrix.sh` together with the fixed hyperparameters in
> `ai/train/run_train.py`. If the two disagree, the script is what ran.

Condition membership and counts are defined in
[`../../../docs/experiment_design.md`](../../../docs/experiment_design.md):
A = G5 x 60, B = (G1, G6, G8) x 20, C = 60 distributed across G1-G9, and
D = G1-G9 x 20 = 180. Materialisation records the exact UUID membership in an
immutable subset manifest, and all three seeds of a condition use the same
manifest.

Every model takes a single three-channel RGB observation. No condition adds a
second sensor channel.

Every run uses 100,000 optimisation steps and saves a checkpoint every 20,000.
Evaluation uses only the final 100,000-step checkpoint. Interrupted runs are
restarted from the beginning, never resumed.
