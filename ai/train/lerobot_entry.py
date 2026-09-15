"""Delegate to the vendored LeRobot trainer for the RGB-only dataset."""

from __future__ import annotations

from lerobot.scripts import lerobot_train

def main():
    lerobot_train.train()


if __name__ == "__main__":
    main()
