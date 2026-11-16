"""Replay window shows the task of flap-labelled episodes: episode list (flap order), the selected episode's flaps
with their time spans, and a "task" panel in the LeRobot / Rerun viewer that follows the playback.

A tiny dataset (2 episodes, one 64x48 camera, per-frame flap tasks + meta/subtask_marks.jsonl) is written with the
LeRobot Python; dataset_info.py / view_episode.py run there, the window test runs a hidden Tk window."""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEROBOT_PY = os.environ.get("LEROBOT_PY") or next(
    (str(p) for p in (ROOT / ".venv_record/bin/python", Path.home() / "Robotic_infra/lerobot/.venv/bin/python")
     if p.exists()), None)
ENV = {**os.environ, "HF_HUB_OFFLINE": "1"}
FLAPS = ["open the front flap", "open the back flap", "open the left flap"]

MAKE = r"""
import json, sys, numpy as np
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset
root = Path(sys.argv[1]); n14 = [f"j{i}" for i in range(14)]
feats = {"observation.state": {"dtype": "float32", "shape": (14,), "names": n14},
         "action": {"dtype": "float32", "shape": (14,), "names": n14},
         "observation.images.head": {"dtype": "video", "shape": (48, 64, 3), "names": ["height", "width", "channels"]}}
ds = LeRobotDataset.create(repo_id="local/flapds", fps=10, features=feats, root=root, robot_type="test", use_videos=True,
                           image_writer_threads=2)
F = json.loads(sys.argv[2])
for ep in (sum(([t] * 10 for t in F), []), [F[0]] * 20):
    for i, t in enumerate(ep):
        ds.add_frame({"observation.state": np.full(14, i, np.float32), "action": np.zeros(14, np.float32),
                      "observation.images.head": np.full((48, 64, 3), (i * 8) % 255, np.uint8), "task": t})
    ds.save_episode(parallel_encoding=False)
ds.finalize()
lines = [{"episode_index": 0, "subtasks": F, "start_frames": [0, 10, 20]}, {"episode_index": 1, "subtasks": F[:1], "start_frames": [0]}]
(root / "meta" / "subtask_marks.jsonl").write_text("".join(json.dumps(l) + "\n" for l in lines))
"""


def run(*args, timeout=300):
    r = subprocess.run([LEROBOT_PY, *map(str, args)], capture_output=True, text=True, env=ENV, timeout=timeout)
    if r.returncode != 0:
        raise AssertionError(r.stdout[-2000:] + r.stderr[-3000:])
    return r.stdout


def dataset_info(path):
    return json.loads(run(ROOT / "recording" / "dataset_info.py", path).strip().splitlines()[-1])


@unittest.skipIf(LEROBOT_PY is None, "needs the LeRobot recorder Python")
class ReplayFlapsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.ds = Path(cls.tmp.name) / "flapds"
        run("-c", MAKE, cls.ds, json.dumps(FLAPS))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_episode_list_shows_the_flap_order_and_times(self):
        info = dataset_info(self.ds)
        e0, e1 = info["episodes"]
        self.assertEqual(e0["task"], "open front → back → left")
        self.assertEqual(e0["flaps"], [{"task": FLAPS[0], "start_s": 0.0, "end_s": 1.0},
                                       {"task": FLAPS[1], "start_s": 1.0, "end_s": 2.0},
                                       {"task": FLAPS[2], "start_s": 2.0, "end_s": 3.0}])
        self.assertEqual(e1["task"], "open front")
        self.assertEqual(e1["flaps"], [{"task": FLAPS[0], "start_s": 0.0, "end_s": 2.0}])

    def test_without_flap_marks_the_task_is_shown_as_before(self):
        plain = Path(self.tmp.name) / "plain"
        shutil.copytree(self.ds, plain)
        (plain / "meta" / "subtask_marks.jsonl").unlink()
        e1 = dataset_info(plain)["episodes"][1]
        self.assertEqual(e1["task"], FLAPS[0])
        self.assertNotIn("flaps", e1)

    def test_viewer_gets_a_task_panel_on_the_video_timeline(self):
        out = Path(self.tmp.name) / "rrd"
        text = run(ROOT / "recording" / "view_episode.py", "--root", self.ds, "--episode-index", 0, "--save", out,
                   "--num-workers", 0)
        self.assertIn("[view] task: open the front flap (0.0 s) -> open the back flap (1.0 s) -> open the left flap (2.0 s)",
                      text)
        rrd = next(out.glob("*.rrd"))
        read = run("-c", (
            "import json, sys, rerun.dataframe as rd\n"
            "rec = rd.load_recording(sys.argv[1])\n"
            "t = rec.view(index='frame_index', contents='/task').select().read_all().to_pydict()\n"
            "img = rec.view(index='frame_index', contents='/observation.images.head').select().read_all()\n"
            "print(json.dumps({'frames': t['frame_index'], 'text': [x[0] for x in t['/task:TextDocument:text']], "
            "'images': img.num_rows}))\n"), rrd)
        got = json.loads(read.strip().splitlines()[-1])
        self.assertEqual(got["frames"], [0, 10, 20])
        self.assertEqual([t.splitlines()[0] for t in got["text"]], [f"## {f}" for f in FLAPS])
        self.assertIn("flap 2 of 3", got["text"][1])
        self.assertEqual(got["images"], 30)  # LeRobot's own viewer content is still there

    @unittest.skipIf(not os.environ.get("DISPLAY"), "needs a display")
    def test_replay_window_lists_and_details_the_flaps(self):
        # The hidden window runs in its own Python process, like the real one (launcher/run_replay_gui.sh): after the
        # camera window test (test_camera_setup) has used Tk in this process, creating this window here segfaults
        # inside Tk (ttk.Checkbutton in a second Tk interpreter; not reproduced in a minimal script).
        r = subprocess.run([sys.executable, "-c", WINDOW, str(ROOT), str(self.ds), json.dumps(dataset_info(self.ds))],
                           capture_output=True, text=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-3000:])
        got = json.loads(r.stdout.strip().splitlines()[-1])
        lines = got["lines"]
        self.assertTrue(lines[0].endswith("open front → back → left"), lines)
        self.assertTrue(lines[1].endswith("open front"), lines)
        self.assertIn("1. open the front flap  0–1 s", got["first"])
        self.assertIn("3. open the left flap  2–3 s", got["first"])
        self.assertEqual(got["second"], "Episode 1:   1. open the front flap  0–2 s")
        self.assertIn(str(ROOT / "recording" / "view_episode.py"), got["view"])
        self.assertEqual(got["view"][-1], "1")  # the selected episode


WINDOW = r"""
import json, sys, tkinter as tk
from pathlib import Path
root_dir, ds, info = Path(sys.argv[1]), Path(sys.argv[2]), json.loads(sys.argv[3])
sys.path.insert(0, str(root_dir / "launcher"))
import replay_window as rw
root = tk.Tk()
root.withdraw()
w = rw.ReplayWindow(root)
w.withdraw()
w._show(ds, info)
out = {"lines": list(w.eps.get(0, "end")), "first": w.l_task.cget("text")}
w.eps.selection_clear(0, "end")
w.eps.selection_set(1)
w._on_select()
out["second"] = w.l_task.cget("text")
started = []
rw.Proc = lambda name, cmd, *a, **k: started.append(cmd) or type("P", (), {"log_path": "-"})()
w.view()
out["view"] = [str(c) for c in started[0]]
root.destroy()
print(json.dumps(out, ensure_ascii=False))
"""


if __name__ == "__main__":
    unittest.main()
