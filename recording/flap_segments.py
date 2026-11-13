"""Flap labels of one recorded episode (box-opening demos) - no ROS / LeRobot imports, so it is unit-testable.

config/flaps_sequence.yaml gives the order of the flaps as instruction sentences:

    1: open the back flap
    2: open the front flap
    3: open the right flap
    4: open the left flap

Used only when the recorder runs with --flaps-sequence. When an episode starts recording, step 1 is already the
current flap. `n` in the recorder terminal means "this flap is finished, go to the next one" (so four flaps need three
presses); the last flap ends when the episode is saved.

Dataset format (same as the --subtasks recorder in additional_references/): every frame's LeRobot `task` is the
sentence of the flap that was current when the frame was recorded, so LeRobot stores it as the frame's task_index and
meta/tasks.parquet lists the sentences. After each saved episode one line is appended to
<dataset>/meta/subtask_marks.jsonl: {"episode_index", "subtasks" (the flaps reached), "start_frames" (frame index
within the episode where each one starts; the first is 0)}. A discarded episode leaves nothing.
"""
import json
import os
from pathlib import Path

DEFAULT_SEQUENCE = Path(__file__).resolve().parent.parent / "config" / "flaps_sequence.yaml"
MARKS_FILE = Path("meta") / "subtask_marks.jsonl"


def load_sequence(path) -> list[str]:
    """Sentences in step order (step 1 first). Raises SystemExit with a readable message when the file is wrong."""
    import yaml

    path = Path(path).expanduser()
    where = f"[rec] ERROR: flaps sequence {path}: "
    try:
        data = yaml.safe_load(path.read_text())
    except OSError as e:
        raise SystemExit(where + f"cannot read it ({e.strerror or e})")
    except yaml.YAMLError as e:
        raise SystemExit(where + f"not valid YAML ({e})")
    if not isinstance(data, dict) or not data:
        raise SystemExit(where + "expected lines like '1: open the back flap' (step number: sentence)")
    steps = {}
    for k, v in data.items():
        try:
            if isinstance(k, bool):
                raise ValueError
            n = int(k)
        except (TypeError, ValueError):
            raise SystemExit(where + f"step number {k!r} is not an integer")
        if n in steps:
            raise SystemExit(where + f"step {n} appears twice")
        if not isinstance(v, str) or not v.strip():
            raise SystemExit(where + f"step {n} is {v!r}; it must be a sentence such as 'open the back flap'")
        steps[n] = " ".join(v.split())
    if sorted(steps) != list(range(1, len(steps) + 1)):
        raise SystemExit(where + f"steps must be 1..{len(steps)} without gaps, got {sorted(steps)}")
    names = [steps[i] for i in range(1, len(steps) + 1)]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise SystemExit(where + f"sentence listed more than once: {', '.join(repr(d) for d in dup)}")
    return names


class FlapTracker:
    """Current flap of the open episode. Frames are 'frames recorded so far' when the key is handled."""

    def __init__(self, sequence):
        self.sequence = list(sequence)
        self.idx = 0          # current step - 1
        self.marks = []       # frame index where each later step starts
        self.active = False

    @property
    def task(self) -> str:
        return self.sequence[self.idx]

    def label(self) -> str:
        return f"flap {self.idx + 1}/{len(self.sequence)}: {self.task}"

    def start(self) -> str:
        """A new episode started recording: flap 1 is the current flap."""
        self.idx, self.marks, self.active = 0, [], True
        return self.label()

    def discard(self):
        self.idx, self.marks, self.active = 0, [], False

    def next(self, frame) -> str:
        """`n`: the current flap is finished; the next one starts at `frame`. Returns the message to print."""
        if not self.active:
            return "[rec] n ignored: no episode is recording"
        if self.idx + 1 >= len(self.sequence):
            return f"[rec] n ignored: already on the last flap ({self.label()}); it ends when you save the episode"
        done = self.label()
        self.idx += 1
        self.marks.append(int(frame))
        return f"[rec] {done} finished -> now {self.label()} (from frame {int(frame)})"

    def finish(self) -> dict:
        """The episode is being saved: the flaps reached and where each starts. Resets for the next episode."""
        rec = {"subtasks": self.sequence[: self.idx + 1], "start_frames": [0] + self.marks}
        self.discard()
        return rec

    def summary(self, rec) -> str:
        msg = f"flaps: {len(rec['subtasks'])} of {len(self.sequence)} reached, starting at frames {rec['start_frames']}"
        if len(rec["subtasks"]) < len(self.sequence):
            msg += "  <- fewer than all flaps: a missed 'n'?"
        return msg


def write_marks(root, episode_index, rec):
    """Append the episode's line to <root>/meta/subtask_marks.jsonl (flushed + fsynced)."""
    path = Path(root) / MARKS_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    line = {"episode_index": int(episode_index), "subtasks": rec["subtasks"], "start_frames": rec["start_frames"]}
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(line, ensure_ascii=False) + "\n")
        f.flush()
        os.fsync(f.fileno())
