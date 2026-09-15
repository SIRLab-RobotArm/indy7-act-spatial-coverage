#!/usr/bin/env python3
"""Convert rollout JSONL into Parquet in the conda environment."""

import argparse
import json

import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_jsonl")
    parser.add_argument("output_parquet")
    args = parser.parse_args()
    rows = []
    with open(args.input_jsonl, encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    frame = pd.DataFrame(rows, columns=(
        "t", "ee_pose", "joint_pos", "gripper_cmd", "autonomy_state"))
    frame.to_parquet(args.output_parquet, index=False)


if __name__ == "__main__":
    main()
