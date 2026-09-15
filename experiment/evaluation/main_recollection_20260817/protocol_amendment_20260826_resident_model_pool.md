# Protocol amendment: one-process resident ACT model pool

Date: 2026-08-26 KST

This retrospective execution amendment was made after 19 eligible
smooth-controller trials and their outcomes were known, before Trial 20. It
must not be described as prospective preregistration. Invalid and interrupted
attempts already present on disk remain preserved under their original IDs.

## Change

Previously, the foreground runner constructed one `RGBActBackend` from the
scheduled checkpoint at the start of every trial and destroyed it afterward.
From Trial 20 onward, one foreground CUDA process sequentially loads and warms
the frozen M01--M12 checkpoints, retains all 12 backends, and listens on the
same `tcp://127.0.0.1:5557` endpoint. Before each ROS evaluation graph starts,
the runner sends the blind `model_code`; the server resets and acknowledges
that exact resident backend. Only the selected backend performs inference.

The 12 `model.safetensors` files are 206,462,144 bytes each (about 2.4GB total
weights), which is small relative to the workstation's approximately 96GB GPU
memory and 123GB system RAM. A single process avoids 12 CUDA contexts and 12
ports. Model loading is paid once per foreground UI session and all models are
released when that session ends.

## Invariance and retention

The following are unchanged:

- all checkpoint files and the frozen M01--M12 mapping;
- condition-specific LeRobot dataset metadata, preprocessing, normalization,
  and postprocessing;
- `RGBActBackend.predict_chunk`, policy reset before each rollout, RGB/state
  observations, action chunks, 30Hz controller, gripper behavior, and safety
  handling;
- the randomized 480-row schedule, targets, outcome rules, and analysis.

Future rollout metadata records
`inference_server=resident_12_model_pool_v1`. The first 19 eligible trials lack
that field because they used per-trial process loading, but they use the same
checkpoints and inference implementation and remain in the main analysis. This
change addresses operator waiting time only; it is not a policy or model
selection change.
