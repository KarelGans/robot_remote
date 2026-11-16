#!/usr/bin/env python3
"""LeRobot's episode viewer (Rerun: camera videos, state / action curves) plus the episode's task as a text panel that
follows the playback - e.g. "open the back flap  (flap 2 of 4)" while the back flap is being opened.

The task is read per frame from the dataset itself (task_index), so it also works without meta/subtask_marks.jsonl.

  <lerobot python> recording/view_episode.py --root /path/to/dataset --episode-index 3 [--save DIR]
"""

import argparse
import glob
from pathlib import Path

import pandas as pd
import rerun as rr
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.scripts.lerobot_dataset_viz import visualize_dataset


def task_changes(root: Path, episode: int) -> list:
    """[(frame within the episode, task)] at frame 0 and wherever the task changes."""
    names = {int(r.task_index): t for t, r in pd.read_parquet(root / "meta" / "tasks.parquet").iterrows()}
    parts = [pd.read_parquet(f, columns=["episode_index", "frame_index", "task_index"])
             for f in sorted(glob.glob(str(root / "data" / "**" / "*.parquet"), recursive=True))]
    df = pd.concat(parts)
    g = df[df.episode_index == episode].sort_values("frame_index")
    out, last = [], None
    for frame, ti in zip(g.frame_index.to_numpy(), g.task_index.to_numpy()):
        if ti != last:
            out.append((int(frame), names[int(ti)]))
            last = ti
    return out


def log_tasks(changes, fps):
    n = len(changes)
    for k, (frame, task) in enumerate(changes, 1):
        rr.set_time("frame_index", sequence=frame)
        rr.set_time("timestamp", timestamp=frame / fps)
        text = f"## {task}" + (f"\n\nflap {k} of {n} (from {frame / fps:.1f} s)" if n > 1 else "")
        rr.log("task", rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", required=True, type=Path)
    p.add_argument("--episode-index", required=True, type=int)
    p.add_argument("--repo-id", default=None, help="default: local/<folder name>")
    p.add_argument("--num-workers", type=int, default=4)
    p.add_argument("--save", type=Path, default=None, help="write an .rrd file to this folder instead of opening a viewer")
    args = p.parse_args()
    root = args.root.expanduser().resolve()
    dataset = LeRobotDataset(args.repo_id or f"local/{root.name}", episodes=[args.episode_index], root=root,
                             tolerance_s=1e-4)
    changes = task_changes(root, args.episode_index)
    print("[view] task: " + " -> ".join(f"{t} ({f / dataset.meta.fps:.1f} s)" for f, t in changes), flush=True)
    rrd = visualize_dataset(dataset, args.episode_index, num_workers=args.num_workers, mode="local",
                            save=args.save is not None, output_dir=args.save)
    log_tasks(changes, dataset.meta.fps)  # same Rerun recording, same timelines as the videos / curves
    rr.disconnect()  # flush (viewer or .rrd file)
    if rrd:
        print(f"[view] saved {rrd}", flush=True)


if __name__ == "__main__":
    main()
