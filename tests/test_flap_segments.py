"""Flap labels in the recorder: --flaps-sequence (config/flaps_sequence.yaml) + `n` (= this flap is finished, go on).

Dataset format = the --subtasks recorder in additional_references/: every frame's LeRobot task is the current flap's
sentence, plus one line per saved episode in meta/subtask_marks.jsonl {"episode_index", "subtasks", "start_frames"}.

Part 1 is pure Python (recording/flap_segments.py). Part 2 runs the real recorder (run_record.sh --source real, its own
LeRobot Python, no cameras) against fake driver / teleop topics, drives it through its terminal keys like the operator
does, and reads the result back with LeRobotDataset. Private ROS domain, see ros_isolation."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import json
import os
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_sys.path.insert(0, str(ROOT / "recording"))
LEROBOT_PY = os.environ.get("LEROBOT_PY") or next(
    (str(p) for p in (ROOT / ".venv_record/bin/python", Path.home() / "Robotic_infra/lerobot/.venv/bin/python")
     if p.exists()), None)
DOMAIN = 86
try:
    import yaml  # noqa: F401
    import flap_segments as fs
except ImportError:  # pragma: no cover
    fs = None
try:
    import rclpy
    import rclpy.executors
    from sensor_msgs.msg import JointState
except ImportError:  # pragma: no cover
    rclpy = None

ARM = [f"joint{i}" for i in range(1, 7)]
NAMES = [f"arm_{s}_{j}" for s in ("left", "right") for j in ARM + ["gripper"]]
SENTENCES = ["open the back flap", "open the front flap", "open the right flap", "open the left flap"]
TASK = "open the box flaps"  # --task: must not end up on any frame when flap labels are on
MARK_KEYS = {"episode_index", "subtasks", "start_frames"}


@unittest.skipIf(fs is None, "needs PyYAML")
class FlapSequenceFileTest(unittest.TestCase):
    def load(self, text):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "seq.yaml"
            p.write_text(text)
            return fs.load_sequence(p)

    def refuses(self, text, *words):
        with self.assertRaises(SystemExit) as cm:
            self.load(text)
        for w in words:
            self.assertIn(w, str(cm.exception))

    def test_project_file_is_valid(self):
        # the operator edits the order per dataset (config/flaps_sequence.yaml); any order of the four flaps is fine
        self.assertEqual(sorted(fs.load_sequence(fs.DEFAULT_SEQUENCE)), sorted(SENTENCES))

    def test_orders_by_step_number_not_by_line(self):
        self.assertEqual(self.load("2: open B\n1: open A\n3:   open   C  \n"), ["open A", "open B", "open C"])

    def test_bad_files_are_refused_with_a_reason(self):
        self.refuses("1: open A\n3: open B\n", "without gaps")
        self.refuses("2: open A\n3: open B\n", "without gaps")
        self.refuses("1: open A\n2: open A\n", "more than once")
        self.refuses("1: open A\n2:\n", "step 2", "sentence")         # empty value
        self.refuses("1: open A\n2: 5\n", "step 2", "sentence")       # a number
        self.refuses("1: open A\n2: yes\n", "step 2", "sentence")     # YAML reads yes as a bool
        self.refuses("1: open A\n2: '  '\n", "step 2", "sentence")
        self.refuses("one: open A\n", "not an integer")
        self.refuses("true: open A\n", "not an integer")
        self.refuses("", "expected lines like")
        self.refuses("- open A\n- open B\n", "expected lines like")
        self.refuses("1: [unclosed\n", "not valid YAML")
        with self.assertRaises(SystemExit) as cm:
            fs.load_sequence("/nonexistent/flaps.yaml")
        self.assertIn("cannot read", str(cm.exception))


@unittest.skipIf(fs is None, "needs PyYAML")
class FlapTrackerTest(unittest.TestCase):
    def test_recording_starts_on_flap_one_and_n_moves_on(self):
        t = fs.FlapTracker(SENTENCES)
        self.assertEqual(t.start(), "flap 1/4: open the back flap")
        self.assertEqual(t.task, "open the back flap")
        self.assertIn("now flap 2/4: open the front flap (from frame 100)", t.next(100))
        self.assertEqual(t.task, "open the front flap")
        t.next(250)
        t.next(400)
        rec = t.finish()
        self.assertEqual(rec, {"subtasks": SENTENCES, "start_frames": [0, 100, 250, 400]})
        self.assertNotIn("missed", t.summary(rec))
        self.assertIn("4 of 4 reached", t.summary(rec))

    def test_no_n_means_flap_one_for_the_whole_episode(self):
        t = fs.FlapTracker(SENTENCES)
        t.start()
        rec = t.finish()
        self.assertEqual(rec, {"subtasks": SENTENCES[:1], "start_frames": [0]})
        self.assertIn("1 of 4", t.summary(rec))
        self.assertIn("missed", t.summary(rec))

    def test_n_on_the_last_flap_and_outside_a_recording_do_nothing(self):
        t = fs.FlapTracker(SENTENCES)
        self.assertIn("no episode is recording", t.next(5))
        t.start()
        for f in (10, 20, 30):
            t.next(f)
        self.assertIn("last flap", t.next(40))
        self.assertEqual(t.task, SENTENCES[-1])
        self.assertEqual(t.finish()["start_frames"], [0, 10, 20, 30])  # still four, no fifth
        self.assertIn("no episode is recording", t.next(60))
        self.assertFalse(hasattr(t, "undo"))

    def test_discard_and_restart_begin_at_step_one_again(self):
        t = fs.FlapTracker(SENTENCES)
        t.start()
        t.next(10)
        t.discard()
        self.assertEqual(t.start(), "flap 1/4: open the back flap")
        self.assertEqual(t.finish()["subtasks"], SENTENCES[:1])

    def test_marks_file_lines(self):
        t = fs.FlapTracker(["打開後蓋", "open the front flap"])
        t.start()
        t.next(30)
        rec = t.finish()
        with tempfile.TemporaryDirectory() as tmp:
            fs.write_marks(tmp, 7, rec)
            fs.write_marks(tmp, 8, {"subtasks": ["打開後蓋"], "start_frames": [0]})  # appends
            text = (Path(tmp) / "meta/subtask_marks.jsonl").read_text(encoding="utf-8")
        rows = [json.loads(ln) for ln in text.splitlines()]
        self.assertEqual(rows, [{"episode_index": 7, "subtasks": ["打開後蓋", "open the front flap"], "start_frames": [0, 30]},
                                {"episode_index": 8, "subtasks": ["打開後蓋"], "start_frames": [0]}])
        self.assertIn("打開後蓋", text)  # written as text, not \u escapes (like the reference recorder)


@unittest.skipIf(rclpy is None or LEROBOT_PY is None or fs is None,
                 "needs ROS 2 rclpy, PyYAML and the LeRobot recorder Python")
class RecorderFlapKeysTest(unittest.TestCase):
    """The operator's keys in the recorder terminal -> per-frame tasks + meta/subtask_marks.jsonl of the saved episodes."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root / "cams.json").write_text(json.dumps({"_comment": "no cameras in this test"}))
        self.seq = self.root / "flaps.yaml"
        self.seq.write_text("".join(f"{i}: {s}\n" for i, s in enumerate(SENTENCES, 1)))

    def start_recorder(self, extra=()):
        env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "LEROBOT_PY": LEROBOT_PY}
        rec = subprocess.Popen([str(ROOT / "recording/run_record.sh"), "--source", "real", "--name", "ds",
                                "--task", TASK, "--root", str(self.root), "--fps", "15",
                                "--camera-config", str(self.root / "cams.json"), "--cameras", *extra],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               env=env)
        lines = []
        threading.Thread(target=lambda: [lines.append(ln.rstrip()) for ln in rec.stdout], daemon=True).start()
        return rec, lines

    def run_session(self, keys_fn, extra_args=()):
        """Fake drivers + teleop action, one recorder; keys_fn(send, wait) plays the operator. Returns output lines."""
        ctx = rclpy.Context()
        rclpy.init(context=ctx, domain_id=DOMAIN)
        node = rclpy.create_node("fake_rig_flaps", context=ctx)
        pubs = {n: node.create_publisher(JointState, t, 10) for n, t in
                (("left", "/left/joint_states"), ("right", "/right/joint_states"),
                 ("action", "/quest_teleop/action"))}
        ex = rclpy.executors.SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        stop = threading.Event()

        def publish():
            while not stop.is_set():
                for s in ("left", "right"):
                    pubs[s].publish(JointState(name=ARM, position=[0.1] * 6))
                pubs["action"].publish(JointState(name=NAMES, position=[0.0] * 14))
                ex.spin_once(timeout_sec=0)
                time.sleep(1 / 30)

        rec, lines = self.start_recorder(extra_args)
        pub_thread = threading.Thread(target=publish, daemon=True)
        pub_thread.start()

        def wait(text, count=1, timeout=90):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if sum(text in ln for ln in lines) >= count:
                    return
                self.assertIsNone(rec.poll(), "recorder exited:\n" + "\n".join(lines[-30:]))
                time.sleep(0.05)
            self.fail(f"no {text!r} (x{count}) in recorder output:\n" + "\n".join(lines[-30:]))

        def send(key):
            rec.stdin.write(key + "\n")
            rec.stdin.flush()

        try:
            wait("[rec] controls")
            time.sleep(1.0)  # discovery
            keys_fn(send, wait)
            send("q")
            self.assertEqual(rec.wait(timeout=60), 0, "\n".join(lines[-30:]))
        finally:
            stop.set()
            pub_thread.join()
            if rec.poll() is None:
                rec.kill()
            rec.stdin.close()
            rec.stdout.close()
            node.destroy_node()
            rclpy.shutdown(context=ctx)
        self.assertFalse(any("Traceback" in ln for ln in lines), "\n".join(lines))
        return lines

    def marks(self):
        p = self.root / "ds/meta/subtask_marks.jsonl"
        return [json.loads(ln) for ln in p.read_text(encoding="utf-8").splitlines()] if p.exists() else []

    def frame_tasks(self):
        """What LeRobot itself reads: {episode: [task of frame 0, 1, ...]}, and the dataset's task list."""
        code = ("import json, sys\n"
                "from lerobot.datasets.lerobot_dataset import LeRobotDataset\n"
                "ds = LeRobotDataset('local/ds', root=sys.argv[1])\n"
                "eps = {}\n"
                "for i in range(len(ds)):\n"
                "    it = ds[i]\n"
                "    eps.setdefault(int(it['episode_index']), []).append((int(it['frame_index']), it['task']))\n"
                "out = {e: [t for _, t in sorted(v)] for e, v in eps.items()}\n"
                "print(json.dumps({'episodes': out, 'tasks': list(ds.meta.tasks.index)}))\n")
        out = subprocess.run([LEROBOT_PY, "-c", code, str(self.root / "ds")], capture_output=True, text=True,
                             env={**os.environ, "HF_HUB_OFFLINE": "1"}, timeout=180)
        self.assertEqual(out.returncode, 0, out.stdout[-2000:] + out.stderr[-3000:])
        d = json.loads(out.stdout.strip().splitlines()[-1])
        return {int(k): v for k, v in d["episodes"].items()}, d["tasks"]

    def assert_frames_follow_marks(self, frames, mark):
        """frames [start_k, start_k+1) carry subtask k; the last one runs to the end of the episode."""
        self.assertEqual(set(mark), MARK_KEYS)
        starts, subtasks = mark["start_frames"], mark["subtasks"]
        self.assertEqual(starts[0], 0)
        self.assertEqual(starts, sorted(set(starts)), mark)  # strictly increasing
        self.assertLess(starts[-1], len(frames))
        ends = starts[1:] + [len(frames)]
        expected = [s for s, a, b in zip(subtasks, starts, ends) for _ in range(a, b)]
        self.assertEqual(frames, expected)

    def test_keys_label_every_frame_with_its_flap(self):
        def episode_keys(send, wait):
            # an n typed before anything is recording does nothing
            send("n")
            wait("n ignored: no episode is recording")
            # episode 0: four flaps, and a stray 5th n
            send("")
            wait("RECORDING episode 0  flap 1/4: open the back flap")
            for i, s in enumerate(SENTENCES[1:], 2):
                time.sleep(0.7)
                send("n")
                wait(f"now flap {i}/4: {s}")
            send("n")
            wait("already on the last flap")
            time.sleep(0.7)
            send("")
            wait("saved. total episodes: 1")
            wait("flaps: 4 of 4 reached")
            # episode 1: no n at all -> flap 1 for the whole episode
            send("")
            wait("RECORDING episode 1")
            time.sleep(1.0)
            send("")
            wait("saved. total episodes: 2")
            wait("flaps: 1 of 4 reached")
            # episode 2: n, then thrown away with d -> leaves nothing; the next episode starts on flap 1 again
            send("")
            wait("RECORDING episode 2")
            time.sleep(0.5)
            send("n")
            wait("now flap 2/4: open the front flap", count=2)  # (episode 0 printed the first one)
            send("d")
            wait("discarded")
            send("")
            wait("RECORDING episode 2  flap 1/4: open the back flap", count=2)
            time.sleep(0.6)
            send("n")
            wait("now flap 2/4: open the front flap", count=3)
            time.sleep(0.6)
            send("")
            wait("saved. total episodes: 3")

        lines = self.run_session(episode_keys, ["--flaps-sequence", str(self.seq)])
        self.assertTrue(any("flaps: 1 of 4 reached" in ln and "missed" in ln for ln in lines))
        marks = self.marks()
        self.assertEqual([m["episode_index"] for m in marks], [0, 1, 2])  # the discarded episode left no line
        frames, tasks = self.frame_tasks()
        self.assertEqual(sorted(frames), [0, 1, 2])
        self.assertEqual(sorted(tasks), sorted(SENTENCES))  # LeRobot's task list = the sentences, not --task
        self.assertNotIn(TASK, tasks)
        self.assertEqual(marks[0]["subtasks"], SENTENCES)
        self.assertEqual(marks[1], {"episode_index": 1, "subtasks": SENTENCES[:1], "start_frames": [0]})
        self.assertEqual(marks[2]["subtasks"], SENTENCES[:2])
        for m in marks:
            self.assert_frames_follow_marks(frames[m["episode_index"]], m)
        # ~0.7 s per flap at 15 fps: every flap of episode 0 really has frames
        self.assertTrue(all(b - a >= 5 for a, b in zip(marks[0]["start_frames"], marks[0]["start_frames"][1:])))

        # a later session on the same dataset (resume) with --flaps-sequence alone = config/flaps_sequence.yaml:
        # next episode index, starts on flap 1 again, earlier lines untouched, no mixing warning
        project = fs.load_sequence(fs.DEFAULT_SEQUENCE)

        def resumed(send, wait):
            send("")
            wait(f"RECORDING episode 3  flap 1/{len(project)}: {project[0]}")
            time.sleep(0.6)
            send("n")
            wait(f"now flap 2/{len(project)}: {project[1]}")
            time.sleep(0.6)
            send("")
            wait("saved. total episodes: 4")

        lines = self.run_session(resumed, ["--flaps-sequence"])
        self.assertTrue(any(str(fs.DEFAULT_SEQUENCE) in ln for ln in lines if "flap order" in ln))
        self.assertFalse(any("WARNING: the" in ln for ln in lines), "\n".join(lines))
        marks2 = self.marks()
        self.assertEqual(marks2[:3], marks)
        self.assertEqual((marks2[3]["episode_index"], marks2[3]["subtasks"]), (3, project[:2]))
        frames, _ = self.frame_tasks()
        self.assert_frames_follow_marks(frames[3], marks2[3])

    def test_without_flaps_sequence_every_frame_gets_the_task(self):
        def keys(send, wait):
            send("")
            wait(f"RECORDING episode 0  (task: {TASK})")
            time.sleep(0.5)
            send("n")
            wait("n ignored: start the recorder with --flaps-sequence")
            time.sleep(0.5)
            send("")
            wait("saved. total episodes: 1")

        lines = self.run_session(keys)
        self.assertFalse(any("flap order" in ln for ln in lines))
        self.assertEqual(self.marks(), [])
        self.assertFalse((self.root / "ds/meta/subtask_marks.jsonl").exists())
        frames, tasks = self.frame_tasks()
        self.assertEqual(tasks, [TASK])
        self.assertTrue(frames[0] and all(t == TASK for t in frames[0]))

        # resuming that dataset with flap labels warns that the episodes would be labelled differently
        lines = self.run_session(lambda send, wait: wait("flap order"), ["--flaps-sequence", str(self.seq)])
        self.assertTrue(any("WARNING: the 1 earlier episodes" in ln and "without flap labels" in ln for ln in lines),
                        "\n".join(lines))

    def test_bad_yaml_stops_the_recorder_before_it_opens_anything(self):
        bad = self.root / "bad.yaml"
        bad.write_text("1: open the back flap\n2: 5\n")
        env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "LEROBOT_PY": LEROBOT_PY}
        out = subprocess.run([str(ROOT / "recording/run_record.sh"), "--source", "real", "--name", "ds", "--task", "t",
                              "--root", str(self.root), "--camera-config", str(self.root / "cams.json"),
                              "--cameras", "--flaps-sequence", str(bad)],
                             capture_output=True, text=True, env=env, timeout=60)
        self.assertNotEqual(out.returncode, 0)
        self.assertIn("step 2", out.stdout + out.stderr)
        self.assertFalse((self.root / "ds").exists())  # nothing was created


if __name__ == "__main__":
    unittest.main()
