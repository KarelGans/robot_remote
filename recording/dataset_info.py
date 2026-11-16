#!/usr/bin/env python3
"""Print a LeRobot dataset's summary and episode list as one JSON line (used by the replay window).

  <lerobot python> recording/dataset_info.py /path/to/dataset

Per episode: index, length, task. A dataset recorded with --flaps-sequence (meta/subtask_marks.jsonl) also gets
"flaps": [{"task", "start_s", "end_s"}, ...] in the order they were opened, and "task" becomes that order, e.g.
"open front -> back -> left -> right". Otherwise "task" is the episode's (first) task as before.
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flap_segments  # noqa: E402


def short(task: str) -> str:
    """'open the back flap' -> 'back' (other sentences unchanged)."""
    s = task.strip()
    if s.lower().startswith("open the "):
        s = s[len("open the "):]
    if s.lower().endswith(" flap"):
        s = s[: -len(" flap")]
    return s


def flap_order_text(tasks) -> str:
    names = [short(t) for t in tasks]
    if all(t.lower().startswith("open the ") for t in tasks):
        return "open " + " → ".join(names)
    return " → ".join(tasks)


def read_marks(root: Path) -> dict:
    p = root / flap_segments.MARKS_FILE
    if not p.is_file():
        return {}
    out = {}
    for line in p.read_text(encoding="utf-8").splitlines():
        if line.strip():
            m = json.loads(line)
            out[int(m["episode_index"])] = m
    return out


def episode_flaps(mark, length, fps) -> list:
    """[{task, start_s, end_s}] of one episode from its subtask_marks line (the last flap runs to the end)."""
    starts = list(mark["start_frames"]) + [length]
    return [{"task": t, "start_s": round(a / fps, 1), "end_s": round(b / fps, 1)}
            for t, a, b in zip(mark["subtasks"], starts, starts[1:])]


def main():
    root = Path(sys.argv[1]).expanduser().resolve()
    if not (root / "meta" / "info.json").is_file():
        print(json.dumps({"error": f"not a LeRobot dataset (no meta/info.json): {root}"}))
        return
    from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata

    try:
        meta = LeRobotDatasetMetadata(f"local/{root.name}", root=root)
    except Exception as e:  # incomplete folder (e.g. moved while the recorder was still finalizing)
        missing = [] if any((root / "meta" / "episodes").glob("*/*.parquet")) else ["meta/episodes/*.parquet"]
        print(json.dumps({"error": f"cannot read the dataset ({type(e).__name__})"
                          + (f"; missing {missing[0]} - was it moved before the recorder finished?" if missing else "")}))
        return
    marks = read_marks(root)
    eps = []
    for row in meta.episodes:
        tasks = row.get("tasks") or []
        ep, length = int(row["episode_index"]), int(row["length"])
        e = {"index": ep, "length": length, "task": tasks[0] if isinstance(tasks, list) and tasks else str(tasks)}
        if ep in marks:
            e["flaps"] = episode_flaps(marks[ep], length, meta.fps)
            e["task"] = flap_order_text(marks[ep]["subtasks"])
        eps.append(e)
    print(json.dumps({"name": root.name, "fps": meta.fps, "robot_type": meta.robot_type,
                      "total_episodes": meta.total_episodes, "total_frames": meta.total_frames,
                      "cameras": [k.split(".")[-1] for k in meta.camera_keys], "episodes": eps}, ensure_ascii=False))


if __name__ == "__main__":
    main()
