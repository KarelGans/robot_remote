"""Camera settings ("controls" in config/real_cameras.json) and the repeated-frame count of recording/real_cameras.py.

The V4L2 ioctls are answered by a fake camera (fcntl.ioctl patched), so no camera is needed; it behaves like a UVC
camera: control classes and disabled controls are listed but skipped, and the exposure time is refused while auto
exposure is on. The repeat count runs on video files standing in for cameras."""
import errno
import os
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "recording"))
try:
    import cv2
    import numpy as np
    import real_cameras as rc
except ImportError:  # pragma: no cover
    rc = None


class FakeUvcCamera:
    """Answers VIDIOC_QUERYCTRL (with NEXT_CTRL) / G_CTRL / S_CTRL like a UVC camera."""

    def __init__(self):
        # id: [name, type, min, max, step, default, flags, value]
        self.ctrls = {
            0x00980001: ["User Controls", 6, 0, 0, 0, 0, 0, 0],                     # class: skipped
            0x00980900: ["Brightness", 1, -64, 64, 1, 0, 0, 0],
            0x00980913: ["Gain", 1, 0, 100, 1, 0, 0, 0],
            0x00980918: ["Power Line Frequency", 3, 0, 2, 1, 1, 0, 1],
            0x00980999: ["Old Control", 1, 0, 1, 1, 0, rc.CTRL_FLAG_DISABLED, 0],   # disabled: skipped
            0x009A0901: ["Auto Exposure", 3, 0, 3, 1, 3, 0, 3],
            0x009A0902: ["Exposure Time, Absolute", 1, 1, 5000, 1, 157, 0, 157],
            0x009A0903: ["Exposure, Dynamic Framerate", 2, 0, 1, 1, 0, 0, 1],
            0x009A0904: ["Focus Step", 1, 0, 100, 10, 0, 0, 0],                     # rounds to its step
        }
        self.sets = []

    def ioctl(self, fd, req, buf, *a):
        if req == rc.VIDIOC_QUERYCTRL:
            cid = struct.unpack_from("<I", buf)[0]
            if cid & rc.CTRL_FLAG_NEXT:
                later = sorted(i for i in self.ctrls if i > (cid & ~rc.CTRL_FLAG_NEXT))
                if not later:
                    raise OSError(errno.EINVAL, "end")
                cid = later[0]
            name, typ, mn, mx, step, default, flags, _ = self.ctrls[cid]
            buf[:] = struct.pack(rc.QUERYCTRL_FMT, cid, typ, name.encode(), mn, mx, step, default, flags, 0, 0)
        elif req == rc.VIDIOC_G_CTRL:
            cid = struct.unpack_from("<I", buf)[0]
            buf[:] = struct.pack("<Ii", cid, self.ctrls[cid][7])
        elif req == rc.VIDIOC_S_CTRL:
            cid, value = struct.unpack("<Ii", buf)
            if cid == 0x009A0902 and self.ctrls[0x009A0901][7] != 1:
                raise OSError(errno.EACCES, "Permission denied")  # exposure time only in manual mode
            step = self.ctrls[cid][4] or 1
            self.ctrls[cid][7] = value - value % step
            self.sets.append((self.ctrls[cid][0], value))
        else:
            raise OSError(errno.ENOTTY, "not a fake ioctl")
        return 0


@unittest.skipIf(rc is None, "needs OpenCV")
class CameraControlsTest(unittest.TestCase):
    def setUp(self):
        self.cam = FakeUvcCamera()
        tmp = tempfile.NamedTemporaryFile(delete=False)
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)
        self.dev = tmp.name  # os.open needs something openable; the ioctls go to the fake
        patcher = mock.patch("fcntl.ioctl", self.cam.ioctl)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_names_like_v4l2_ctl_and_skipped_entries(self):
        self.assertEqual(rc.control_name(b"Exposure Time, Absolute\0\0junk"), "exposure_time_absolute")
        self.assertEqual(rc.control_name(b"Exposure, Dynamic Framerate"), "exposure_dynamic_framerate")
        ctrls = rc.list_controls(self.dev)
        self.assertEqual(list(ctrls), ["brightness", "gain", "power_line_frequency", "auto_exposure",
                                       "exposure_time_absolute", "exposure_dynamic_framerate", "focus_step"])
        self.assertEqual(ctrls["exposure_time_absolute"],
                         {"id": 0x009A0902, "value": 157, "min": 1, "max": 5000, "step": 1, "default": 157,
                          "inactive": False})

    def test_mode_switches_first_so_the_exposure_is_accepted(self):
        wanted = {"exposure_time_absolute": 167, "auto_exposure": 1, "power_line_frequency": 2,
                  "exposure_dynamic_framerate": 0}
        applied, problems = rc.set_controls(self.dev, wanted)
        self.assertEqual(problems, [])
        self.assertEqual(applied, {"power_line_frequency": 2, "auto_exposure": 1, "exposure_dynamic_framerate": 0,
                                   "exposure_time_absolute": 167})
        self.assertEqual([n for n, _ in self.cam.sets], ["Power Line Frequency", "Auto Exposure",
                                                         "Exposure, Dynamic Framerate", "Exposure Time, Absolute"])

    def test_problems_are_reported_and_the_rest_still_applied(self):
        applied, problems = rc.set_controls(self.dev, {
            "exposure_time_absolute": 167,  # auto exposure stays on -> refused
            "shutter_speed": 5,             # no such control
            "gain": 500,                    # out of range
            "brightness": "dark",           # not a number
            "power_line_frequency": True,   # a bool is not a setting
            "focus_step": 25,               # the camera rounds it to its step
        })
        text = "\n".join(problems)
        self.assertIn("exposure_time_absolute=167: refused by the camera", text)
        self.assertIn("shutter_speed: this camera has no such setting", text)
        self.assertIn("auto_exposure", text)  # lists what it does have
        self.assertIn("gain=500: outside 0..100", text)
        self.assertIn("brightness: 'dark' is not a whole number", text)
        self.assertIn("power_line_frequency: True is not a whole number", text)
        self.assertIn("focus_step: set 25, the camera reports 20", text)
        self.assertEqual(applied, {"focus_step": 20})
        self.assertEqual(self.cam.ctrls[0x00980913][7], 0)  # gain untouched


def make_video(path, moving):
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (160, 120))
    rng = np.random.default_rng(1)
    still = rng.integers(0, 255, (120, 160, 3), np.uint8)
    for _ in range(30):
        w.write(rng.integers(0, 255, (120, 160, 3), np.uint8) if moving else still)
    w.release()


@unittest.skipIf(rc is None, "needs OpenCV")
class RepeatedFramesTest(unittest.TestCase):
    def test_identical_frames_are_counted(self):
        with tempfile.TemporaryDirectory() as tmp:
            counts = {}
            for name, moving in (("still", False), ("moving", True)):
                path = Path(tmp) / f"{name}.avi"
                make_video(path, moving)
                cam = rc.RealCamera(name, str(path), 160, 120, 60, None,
                                    controls={"auto_exposure": 1}).open()  # settings are skipped for files
                time.sleep(1.0)
                cam.close()
                self.assertEqual(cam.control_problems, [])
                counts[name] = (cam.frames, cam.repeats)
        frames, repeats = counts["still"]
        self.assertGreater(frames, 20)
        self.assertGreater(repeats / frames, 0.9)  # (each restart of the looping file may count as a new picture)
        frames, repeats = counts["moving"]
        self.assertGreater(frames, 20)
        self.assertEqual(repeats, 0)


if __name__ == "__main__":
    unittest.main()
