"""real_robot/mocap_follower.py: rebel_mocap_teleop (JointState on the mocap domain) -> fake ReBeL driver.

Robot side on the isolated test domain (ros_isolation, 97), mocap side on domain 98; the fake driver is the ReBeL
plant of test_real_ros_tracking (JTC velocity interface + identified arm dynamics). Checks: follows a target and feeds
the measured joints back with rebel_demo's names; holds on a stale stream, on a jump and when disabled; clamps the joint
limits; refuses to start next to another command publisher or with a robot domain as the mocap domain.
"""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402  - FIRST: never share the robot's ROS graph
import math
import os
import subprocess
import threading
import time
import unittest
from pathlib import Path

try:
    import numpy as np
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from test_real_ros_tracking import FakeRebelDriver
except ImportError:  # pragma: no cover
    rclpy = None

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "real_robot" / "run_mocap_follower.sh"
MOCAP_DOMAIN = 98
HOME = [0.0, 0.0, 65.0, 0.0, 50.0, 0.0]
NAMES = [f"arm_left_joint{i}" for i in range(1, 7)] + [f"arm_right_joint{i}" for i in range(1, 7)]


@unittest.skipIf(rclpy is None, "needs ROS 2 (source /opt/ros/humble/setup.bash)")
class MocapFollowerTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        ros_isolation.check(os.environ["ROS_DOMAIN_ID"])
        self.drv = FakeRebelDriver(["left"], HOME)
        self.traj_t = []
        self.drv.node.create_subscription(JointTrajectory, "/left/rebel_arm_trajectory_controller/joint_trajectory",
                                          lambda m: self.traj_t.append(time.monotonic()), 50)
        self.ctx = Context()  # the fake rebel_demo is LAN-visible like the real mocap PC (98 is no robot domain)
        os.environ["ROS_LOCALHOST_ONLY"] = "0"
        try:
            rclpy.init(context=self.ctx, domain_id=MOCAP_DOMAIN)
        finally:
            os.environ["ROS_LOCALHOST_ONLY"] = "1"
        self.mn = rclpy.create_node("fake_rebel_demo", context=self.ctx)
        self.fb = None
        self.mn.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "fb", m), 10)
        self.pub = self.mn.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.mn)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.proc = None

    def tearDown(self):
        if self.proc and self.proc.poll() is None:
            self.proc.stdin.write("q\n")
            self.proc.stdin.flush()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.proc:
            out = self.proc.stdout.read()
            self.assertNotIn("Traceback", out, out)
        self.ex.shutdown(timeout_sec=1.0)
        self.mn.destroy_node()
        rclpy.shutdown(context=self.ctx)
        self.drv.close()
        rclpy.try_shutdown()

    def start(self, *extra, enable=True):
        args = [str(RUN), "--arm", "left", "--mocap-domain", str(MOCAP_DOMAIN), "--trace", os.environ.get("MF_TEST_TRACE", ""), *extra]
        self.proc = subprocess.Popen(args + (["--enable"] if enable else []), stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        end = time.monotonic() + 15
        while self.fb is None and time.monotonic() < end:
            time.sleep(0.05)
        self.assertIsNotNone(self.fb, "no /joint_states feedback on the mocap domain")

    def meas(self):
        return np.rad2deg(self.drv.q["left"]).copy()

    def send(self, q_deg, secs, hz=60):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.pub.publish(JointState(name=NAMES, position=list(np.deg2rad(q_deg)) + [0.0] * 6))
            time.sleep(1 / hz)

    def ramp(self, a, b, secs, hz=60):
        n = int(secs * hz)
        for k in range(1, n + 1):
            self.send(np.array(a) + (np.array(b) - np.array(a)) * k / n, 1 / hz, hz)

    def test_follows_and_feeds_back(self):
        self.start()
        self.assertEqual(list(self.fb.name), NAMES)  # both arms: rebel_demo rig:=both waits for both
        np.testing.assert_allclose(np.rad2deg(self.fb.position[:6]), HOME, atol=0.5)
        tgt = np.array(HOME)
        tgt[2] += 20
        self.send(HOME, 1.0)  # target on the arm first: the follower engages only there
        self.ramp(HOME, tgt, 2.0)
        self.send(tgt, 4.0)
        self.assertLess(np.max(np.abs(self.meas() - tgt)), 1.0)

    def test_stale_stream_holds(self):
        self.start()
        tgt = np.array(HOME)
        tgt[2] += 15
        self.ramp(HOME, tgt, 2.0)
        self.send(tgt, 3.0)
        a = self.meas()
        time.sleep(2.0)  # stream stops
        self.assertLess(np.max(np.abs(self.meas() - a)), 1.0)
        n = len(self.traj_t)
        time.sleep(1.0)
        self.assertEqual(len(self.traj_t), n, "still sending trajectories on a stale stream")

    def test_jump_holds(self):
        self.start()
        self.send(HOME, 1.0)
        jump = np.array(HOME)
        jump[1] += 30
        a = self.meas()
        self.send(jump, 2.0)
        self.assertLess(np.max(np.abs(self.meas() - a)), 1.0)

    def test_disabled_sends_nothing(self):
        self.start(enable=False)
        tgt = np.array(HOME)
        tgt[2] += 15
        self.ramp(HOME, tgt, 2.0)
        self.send(tgt, 1.0)
        self.assertEqual(self.traj_t, [])
        self.proc.stdin.write("e\n")  # enable from stdin; mocap brings the target back to the arm -> follows
        self.proc.stdin.flush()
        end = time.monotonic() + 5  # until it follows (a fixed 0.5 s raced the stdin thread: target left too early)
        while not self.traj_t and time.monotonic() < end:
            self.send(HOME, 0.1)
        self.assertTrue(self.traj_t, "not following after `e` with the target on the arm")
        self.ramp(HOME, tgt, 2.0)
        self.send(tgt, 3.0)
        self.assertLess(abs(self.meas()[2] - tgt[2]), 1.0)

    def test_enable_never_moves_the_arm_to_a_far_target(self):
        # 2026-10-25 on the real arm: `e` with rebel_demo paused at a pose ~20 deg away moved the arm there by itself
        self.start(enable=False)
        far = np.array(HOME) + np.array([-16.6, -4.6, 19.2, -5.0, 16.3, 20.0])
        self.send(far, 1.0)
        self.proc.stdin.write("e\n")
        self.proc.stdin.flush()
        a = self.meas()
        self.send(far, 3.0)
        self.assertEqual(self.traj_t, [], "commands sent toward a far target")
        self.assertLess(np.max(np.abs(self.meas() - a)), 0.2)
        self.ramp(far, HOME, 2.0)  # mocap moves its target back onto the arm -> engages, follows from there
        tgt = np.array(HOME)
        tgt[2] += 10
        self.ramp(HOME, tgt, 1.5)
        self.send(tgt, 3.0)
        self.assertGreater(len(self.traj_t), 0)
        self.assertLess(abs(self.meas()[2] - tgt[2]), 1.0)

    def test_joint_limit_is_clamped(self):
        self.start("--max-joint-vel", "0.5")
        tgt = np.array(HOME)
        tgt[4] = 120.0  # J5 limit 94, margin 3
        self.send(HOME, 1.0)  # target on the arm first: the follower engages only there
        self.ramp(HOME, tgt, 4.0)
        self.send(tgt, 5.0)
        self.assertLess(self.meas()[4], 91.5)
        self.assertGreater(self.meas()[4], 88.0)

    def test_refuses_next_to_another_command_publisher(self):
        other = self.drv.node.create_publisher(JointTrajectory, "/left/rebel_arm_trajectory_controller/joint_trajectory", 10)
        try:
            p = subprocess.run([str(RUN), "--arm", "left", "--mocap-domain", str(MOCAP_DOMAIN), "--trace", ""],
                               input="", capture_output=True, text=True, timeout=30)
            self.assertNotEqual(p.returncode, 0)
            self.assertIn("refusing to start", p.stdout + p.stderr)
        finally:
            self.drv.node.destroy_publisher(other)

    def test_second_joint_state_publisher_stops_the_commands(self):  # mixed feedback (seen with a leftover driver)
        self.start()
        tgt = np.array(HOME)
        tgt[2] += 10
        self.ramp(HOME, tgt, 1.0)
        other = self.drv.node.create_publisher(JointState, "/left/joint_states", 10)
        try:
            time.sleep(1.5)  # discovery + the 0.5 s check
            n = len(self.traj_t)
            stop = threading.Event()

            def zeros():
                while not stop.is_set():
                    other.publish(JointState(name=[f"joint{i}" for i in range(1, 7)], position=[0.0] * 6))
                    time.sleep(0.01)
            threading.Thread(target=zeros, daemon=True).start()
            a = self.meas()
            self.send(tgt + np.array([0, 0, 15, 0, 0, 0]), 2.0)
            stop.set()
            self.assertEqual(len(self.traj_t), n, "commands sent with two joint_states publishers")
            self.assertLess(np.max(np.abs(self.meas() - a)), 1.5)
        finally:
            self.drv.node.destroy_publisher(other)

    def test_arm_that_stops_following_stops_the_commands(self):
        # 2026-10-25 on the real arm: J5 overcurrent -> motors off; the node then kept sending for 40 s in "glide"
        # (an old glide goal + the leash kept the command velocity above zero forever)
        self.start()
        tgt = np.array(HOME)
        tgt[2] += 15
        self.send(HOME, 1.0)  # target on the arm first: the follower engages only there
        self.ramp(HOME, tgt, 1.5)  # moving: leaves a glide goal behind
        self.send(tgt, 0.5)
        self.drv.frozen = True     # motors off
        far = tgt.copy()
        far[2] += 10
        self.ramp(tgt, far, 1.0)
        self.send(far, 3.0)        # stall detected after 1 s
        n = len(self.traj_t)
        self.send(far, 2.0)
        self.assertEqual(len(self.traj_t), n, "still sending to an arm that does not follow")
        self.assertIn("arm does not follow", self.proc_log())

    def test_recorder_topics(self):
        got = {}
        self.drv.node.create_subscription(JointState, "/quest_teleop/action", lambda m: got.__setitem__("a", m), 10)
        self.drv.node.create_subscription(JointState, "/quest_teleop/joint_states", lambda m: got.__setitem__("j", m), 10)
        self.start("--other-arm-deg", "0", "0", "65", "0", "50", "0")
        q = np.deg2rad(HOME)
        closed = -0.0116631
        names = [f"arm_left_joint{i}" for i in range(1, 7)] + ["arm_left_xeg32_left_carriage_joint"]
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.pub.publish(JointState(name=names, position=list(q) + [closed]))
            time.sleep(1 / 60)
        a = dict(zip(got["a"].name, got["a"].position))
        self.assertEqual(len(got["a"].name), 14)
        self.assertAlmostEqual(a["arm_left_gripper"], 1.0, places=3)
        self.assertAlmostEqual(a["arm_right_gripper"], 0.0)
        np.testing.assert_allclose(np.rad2deg([a[f"arm_left_joint{i}"] for i in range(1, 7)]), HOME, atol=0.5)
        np.testing.assert_allclose(np.rad2deg([a[f"arm_right_joint{i}"] for i in range(1, 7)]), HOME, atol=1e-6)
        j = dict(zip(got["j"].name, got["j"].position))
        np.testing.assert_allclose(np.rad2deg([j[f"arm_right_joint{i}"] for i in range(1, 7)]), HOME, atol=1e-6)
        np.testing.assert_allclose(np.rad2deg([j[f"arm_left_joint{i}"] for i in range(1, 7)]), self.meas(), atol=0.5)

    def home_service(self):
        """Fake rebel_demo ~/home: counts the calls."""
        from std_srvs.srv import Trigger
        calls = []

        def on_home(req, res):
            calls.append(time.monotonic())
            res.success, res.message = True, "moving to HOME (disengaged)"
            return res
        self.mn.create_service(Trigger, "/rebel_teleop/home", on_home)
        return calls

    def episode_end(self, what):
        from std_msgs.msg import String
        pub = self.drv.node.create_publisher(String, "/recorder/episode_end", 10)
        time.sleep(0.5)  # discovery
        pub.publish(String(data=what))

    def to_work(self):
        """Follow the mocap target from HOME to a work pose; returns that pose [deg]."""
        work = np.array(HOME) + [12.0, 6.0, -10.0, 0.0, 8.0, 0.0]
        self.send(HOME, 1.0)  # target on the arm first: the follower engages only there
        self.ramp(HOME, work, 2.0)
        self.send(work, 2.0)
        np.testing.assert_allclose(self.meas(), work, atol=1.0)
        return work

    def test_episode_end_homes_the_arm_itself(self):
        calls = self.home_service()
        self.start("--home-deg", *map(str, HOME))
        work = self.to_work()
        self.episode_end("saved")
        self.send(work, 4.0)  # the mocap target stays at the work pose: the home move ignores it
        np.testing.assert_allclose(self.meas(), HOME, atol=1.0)
        self.assertEqual(len(calls), 1, "rebel_demo ~/home not called (it resumes from its own last command)")
        n = len(self.traj_t)
        self.send(work, 1.0)  # at home, a target 12 deg away is not followed
        np.testing.assert_allclose(self.meas(), HOME, atol=1.0)
        self.assertLessEqual(len(self.traj_t) - n, 1)
        self.send(HOME, 1.0)  # rebel_demo's target home too -> follows again
        self.ramp(HOME, work, 2.0)
        self.send(work, 2.0)
        np.testing.assert_allclose(self.meas(), work, atol=1.0)
        log = self.proc_log()
        self.assertIn("moving home (episode saved)", log)
        self.assertIn("at home", log)

    def test_rebel_demo_pause_does_not_home(self):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        calls = self.home_service()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        engaged = self.mn.create_publisher(Bool, "/rebel_teleop/engaged", latched)
        self.start("--home-deg", *map(str, HOME))
        engaged.publish(Bool(data=True))
        work = self.to_work()
        engaged.publish(Bool(data=False))  # SPACE: pause -> rebel_demo holds its target, so does the arm
        self.send(work, 3.0)
        np.testing.assert_allclose(self.meas(), work, atol=1.0)
        self.assertEqual(calls, [])
        self.assertNotIn("moving home", self.proc_log())

    def test_no_home_after_episode_but_h_homes(self):
        self.home_service()
        self.start("--home-deg", *map(str, HOME), "--no-home-after-episode")
        work = self.to_work()
        self.episode_end("discarded")
        self.send(work, 1.5)
        np.testing.assert_allclose(self.meas(), work, atol=1.0)
        self.proc.stdin.write("h\n")
        self.proc.stdin.flush()
        self.send(work, 4.0)
        np.testing.assert_allclose(self.meas(), HOME, atol=1.0)

    def test_disable_stops_a_home_move(self):
        self.start("--home-deg", *map(str, HOME), "--home-vel-deg", "4")  # no rebel_demo ~/home: warns, still homes
        work = self.to_work()
        self.proc.stdin.write("h\n")
        self.proc.stdin.flush()
        self.send(work, 1.0)
        self.proc.stdin.write("d\n")
        self.proc.stdin.flush()
        self.send(work, 2.0)
        n, q = len(self.traj_t), self.meas()
        self.send(work, 2.0)
        self.assertEqual(len(self.traj_t), n, "still sending after d")
        np.testing.assert_allclose(self.meas(), q, atol=0.3)
        self.assertGreater(float(np.max(np.abs(q - work))), 1.0)   # it had started moving home ...
        self.assertGreater(float(np.max(np.abs(q - HOME))), 3.0)   # ... and stopped on the way
        log = self.proc_log()
        self.assertIn("home stopped", log)
        self.assertIn("not available", log)

    def proc_log(self):
        self.proc.stdin.write("q\n")
        self.proc.stdin.flush()
        self.proc.wait(10)
        out = self.proc.stdout.read()
        self.proc_out = out
        return out

    def test_refuses_a_robot_domain_as_mocap_domain(self):
        p = subprocess.run([str(RUN), "--arm", "left", "--mocap-domain", "42", "--trace", ""],
                           input="", capture_output=True, text=True, timeout=30)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("robot domain", p.stdout + p.stderr)


if __name__ == "__main__":
    unittest.main()
