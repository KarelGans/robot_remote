"""Camera setup window (recording/camera_setup_gui.py): find cameras, name them, Save -> config/real_cameras.json.

Logic tests need no camera. The window test uses video files as cameras (RealCamera plays a file in a loop) and a
hidden Tk window, like the launcher tests."""
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "recording"))
try:
    import cv2
    import numpy as np
    import camera_setup_gui as cs
    import real_cameras as rc
except ImportError:  # pragma: no cover
    cs = None

ARC = "05a3:9230"


def node(dev, port, interface=0, serial="", product="HD USB Camera", id_=ARC):
    return {"dev": dev, "port": port, "interface": interface, "serial": serial, "product": product, "id": id_,
            "speed": "480"}


def old_config():
    return {
        "wrist_left": {"label": "Left wrist (ARC USB camera)", "usb_port": "3-9.2", "usb_id": ARC,
                       "fourcc": "MJPG", "width": 640, "height": 480, "fps": 120, "decode_fps": 60},
        "wrist_right": {"label": "Right wrist (ARC USB camera)", "usb_port": "3-9.3", "usb_id": ARC,
                        "fourcc": "MJPG", "width": 640, "height": 480, "fps": 120, "decode_fps": 60,
                        "controls": {"power_line_frequency": 2, "auto_exposure": 1, "exposure_time_absolute": 167}},
        "head": {"label": "Third-person (Gemini 336)", "serial": "CP96563000LY", "interface": 4,
                 "fourcc": "MJPG", "width": 640, "height": 480, "fps": 30},
        "knife": {"label": "Knife close-up (Gemini 305)", "serial": "CV27561000GA", "interface": 4,
                  "fourcc": "MJPG", "width": 640, "height": 480, "fps": 30, "rotate": 90},
    }


@unittest.skipIf(cs is None, "needs OpenCV and tkinter")
class CameraSetupLogicTest(unittest.TestCase):
    def setUp(self):
        self.a = node("/dev/video0", "3-9.2")
        self.b = node("/dev/video2", "3-9.3")
        self.head = node("/dev/video4", "4-4", 4, "CP96563000LY", "Orbbec Gemini 336", "2bc5:0803")
        self.nodes = [self.a, self.b, self.head]
        self.cfg = old_config()

    def test_current_names_follow_the_config(self):
        self.assertEqual(cs.current_names(self.cfg, self.nodes),
                         {"/dev/video0": "wrist_left", "/dev/video2": "wrist_right", "/dev/video4": "head"})

    def test_one_colour_stream_per_camera(self):
        k4 = node("/dev/video6", "4-3", 4, "CV27561000GA", "Orbbec Gemini 305", "2bc5:0840")
        k6 = node("/dev/video8", "4-3", 6, "CV27561000GA", "Orbbec Gemini 305", "2bc5:0840")
        u2 = node("/dev/video12", "1-1", 2, "X1", "Other cam", "1234:0001")
        u0 = node("/dev/video10", "1-1", 0, "X1", "Other cam", "1234:0001")
        nodes = [k6, k4, u2, u0, self.a]
        shown = cs.find_cameras(self.cfg, nodes=nodes)
        self.assertEqual(sorted(n["dev"] for n in shown), ["/dev/video0", "/dev/video10", "/dev/video6"])
        # the 305: its configured interface 4, never 6; the unconfigured camera: its lowest interface
        self.assertEqual(len(cs.find_cameras(self.cfg, all_interfaces=True, nodes=nodes)), 5)

    def test_swapped_wrist_names_keep_each_cameras_settings(self):
        chosen = {"/dev/video0": "wrist_right", "/dev/video2": "wrist_left", "/dev/video4": "head"}
        new = cs.build_config(self.cfg, self.nodes, chosen, {})
        self.assertEqual(new["wrist_left"]["usb_port"], "3-9.3")
        self.assertEqual(new["wrist_right"]["usb_port"], "3-9.2")
        self.assertEqual((new["wrist_left"]["fps"], new["wrist_left"]["decode_fps"]), (120, 60))
        self.assertEqual(new["wrist_left"]["label"], "Left wrist (ARC USB camera)")  # same camera model: label kept
        # camera settings belong to the physical camera (port 3-9.3): they move with it to its new name
        self.assertEqual(new["wrist_left"]["controls"], self.cfg["wrist_right"]["controls"])
        self.assertNotIn("controls", new["wrist_right"])
        self.assertEqual(new["head"]["serial"], "CP96563000LY")
        self.assertEqual(new["knife"], self.cfg["knife"])  # not connected now: kept unchanged

    def test_not_used_removes_the_name_and_duplicates_are_refused(self):
        chosen = {"/dev/video0": "wrist_left", "/dev/video2": "wrist_right", "/dev/video4": cs.NOT_USED}
        new = cs.build_config(self.cfg, self.nodes, chosen, {})
        self.assertNotIn("head", new)
        self.assertIn("knife", new)
        with self.assertRaises(ValueError) as cm:
            cs.build_config(self.cfg, self.nodes, {"/dev/video0": "head", "/dev/video4": "head"}, {})
        self.assertIn("head", str(cm.exception))

    def test_name_on_another_camera_model_gets_that_cameras_settings(self):
        # 'head' moved from the Orbbec to an ARC: ARC settings (from the wrist entries), new label, ARC port
        chosen = {"/dev/video0": "head", "/dev/video2": "wrist_right", "/dev/video4": cs.NOT_USED}
        new = cs.build_config(self.cfg, self.nodes, chosen, {})
        self.assertEqual(new["head"]["usb_port"], "3-9.2")
        self.assertNotIn("serial", new["head"])
        self.assertEqual(new["head"]["fps"], 120)
        self.assertEqual(new["head"]["label"], "head (HD USB Camera)")
        self.assertNotIn("wrist_left", new)  # the camera that had it is now 'head'
        # a format that really gave a picture in the window wins over the guess
        fmt = {"fourcc": "YUYV", "width": 640, "height": 480, "fps": 30}
        self.assertEqual(cs.build_config(self.cfg, self.nodes, chosen, {"/dev/video0": fmt})["head"]["fourcc"], "YUYV")

    def test_format_candidates(self):
        cands = cs.format_candidates(self.cfg, self.a)
        self.assertEqual(cands[0], {k: self.cfg["wrist_left"][k] for k in cs.FORMAT_KEYS if k in self.cfg["wrist_left"]})
        self.assertEqual(len(cands), len({json.dumps(c, sort_keys=True) for c in cands}))  # no repeats
        unknown = node("/dev/video9", "1-1", 0, "", "New cam", "9999:0001")
        self.assertEqual(cs.format_candidates(self.cfg, unknown), cs.FALLBACK_FORMATS)
        # the Orbbec keeps its rotate setting
        knife = node("/dev/video6", "4-3", 4, "CV27561000GA", "Orbbec Gemini 305", "2bc5:0840")
        self.assertEqual(cs.format_candidates(self.cfg, knife)[0]["rotate"], 90)

    def test_save_writes_a_backup_and_a_file_the_recorder_reads(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "real_cameras.json"
            path.write_text("{\"old\": {\"device\": \"/dev/video0\"}}\n")
            old = path.read_bytes()
            new = cs.build_config(self.cfg, self.nodes,
                                  {"/dev/video0": "wrist_right", "/dev/video2": "wrist_left", "/dev/video4": "head"}, {})
            backup = cs.save_config(path, new)
            self.assertEqual(backup.read_bytes(), old)
            self.assertEqual(rc.load_config(path), new)
            self.assertIn("_comment", json.loads(path.read_text()))
            self.assertEqual(len(path.read_text().splitlines()), len(new) + 3)  # one line per camera
            self.assertFalse((Path(tmp) / "real_cameras.json.tmp").exists())


def make_video(path, colour):
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (640, 480))
    for i in range(30):
        f = np.full((480, 640, 3), colour, np.uint8)
        f[:, i * 20:i * 20 + 20] = 255
        w.write(f)
    w.release()


@unittest.skipIf(cs is None or not os.environ.get("DISPLAY"), "needs tkinter, OpenCV and a display")
class CameraSetupWindowTest(unittest.TestCase):
    def test_pictures_swap_and_save(self):
        import tkinter as tk

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            for name, colour in (("a", (0, 0, 200)), ("b", (0, 200, 0)), ("h", (200, 0, 0))):
                make_video(tmp / f"{name}.mp4", colour)
            nodes = [node(str(tmp / "a.mp4"), "3-9.2"), node(str(tmp / "b.mp4"), "3-9.3"),
                     node(str(tmp / "h.mp4"), "4-4", 4, "CP96563000LY", "Orbbec Gemini 336", "2bc5:0803")]
            path = tmp / "real_cameras.json"
            cs.save_config(path, old_config())
            root = tk.Tk()
            root.withdraw()
            app = cs.CameraSetup(root, path, find=lambda cfg, all_interfaces: nodes)
            try:
                end = time.monotonic() + 20
                while time.monotonic() < end and not all(t.photo for t in app.tiles):
                    root.update()
                    time.sleep(0.02)
                self.assertTrue(all(t.photo for t in app.tiles), [t.error for t in app.tiles])
                a, b, h = app.tiles
                self.assertEqual([t.var.get() for t in app.tiles], ["wrist_left", "wrist_right", "head"])
                self.assertIn("knife", app.kept.cget("text"))  # configured but not connected
                self.assertIn("MJPG 640x480 @ 120", a.fmt_label.cget("text"))
                b.var.set("wrist_left")  # pick the other wrist's name -> the two swap
                app.on_pick(b)
                self.assertEqual((a.var.get(), b.var.get()), ("wrist_right", "wrist_left"))
                self.assertTrue(app.dirty)
                self.assertTrue(app.save())
                self.assertFalse(app.dirty)
                cfg = rc.load_config(path)
                self.assertEqual((cfg["wrist_left"]["usb_port"], cfg["wrist_right"]["usb_port"]), ("3-9.3", "3-9.2"))
                self.assertEqual(cfg["knife"], old_config()["knife"])
                self.assertEqual(rc.load_config(path.with_name("real_cameras.json.bak")), old_config())
                self.assertIn("Saved", app.status.cget("text"))
            finally:
                app.close()  # not dirty after Save: no question asked
            self.assertTrue(all(t.cam is None for t in app.tiles))


if __name__ == "__main__":
    unittest.main()
