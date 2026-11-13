#!/usr/bin/env python3
"""Mocap teleop (KarelGans/rebel_mocap_teleop, ROS package rebel_demo) -> one physical ReBeL through the igus ROS driver.

rebel_demo runs on another PC (Wi-Fi) on its own ROS domain (default here 12) and publishes ONE sensor_msgs/JointState
at ~60 Hz on /dual_arm/isaac_joint_commands: absolute positions [rad] of arm_left_joint1..6 / arm_right_joint1..6 (+
gripper carriages, ignored: no physical gripper). It expects the robot's measured joints back on /joint_states
(arm_<side>_joint1..6; start it with robot_joint_states_topic:=/joint_states require_joint_states:=true).

This node is the bridge (it replaces the group's MoveIt Servo "mocap follower"). One process, two ROS contexts:
  mocap domain (LAN, not localhost-only):  sub /dual_arm/isaac_joint_commands   pub /joint_states, /mocap_follower/status
  robot domain (ros_env.sh: 42, localhost): sub /<side>/joint_states            pub /<side>/rebel_arm_trajectory_controller/joint_trajectory
Commands go out like the Quest teleop's (one short single-point trajectory per tick) with the limits tuned on the real
arm (2026-10-21): joint speed / acceleration / braking deceleration, command within --max-cmd-lead-deg of the measured
joints, joint limits minus a margin. It HOLDS (glides to a stop, then sends nothing) when the mocap stream is stale or
jumps, the joint states are stale, the arm does not follow, another node publishes on the arm's command topic, or it is
disabled. It starts DISABLED: stdin `e` = enable, `d` = disable, `h` = home, `q` = quit (or std_msgs/Bool on
/mocap_follower/enable on the robot domain, this PC only). There is NO collision checking (other arm, table, stand): keep
the e-stop in reach.
Home (--home-deg, = rebel_demo's home_deg): when an episode stops in the recorder (saved or discarded:
/recorder/episode_end from recording/record_dataset.py) or on stdin `h` - never on a rebel_demo pause - this node
drives the arm home itself, ignoring the mocap target (--home-vel-deg, same acceleration / lead / stall limits), and calls rebel_demo's ~/home so its target - the reference it resumes from - goes home too. Only while enabled;
`d`, a fault or another command publisher stops it. Once the mocap target is back on the arm it follows again: press
SPACE in teleop_keyboard to engage for the next episode.

  real_robot/run_mocap_follower.sh --arm left [--dry-run] [--mocap-domain 12]
"""
import argparse
import csv
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
# Real ReBeL limits (igus_rebel_description/urdf/igus_rebel2.urdf = rebel_mocap_teleop kinematics.py) [deg]
LIMITS_DEG = np.array([[-178, 178], [-79, 139], [-79, 139], [-178, 178], [-94, 94], [-178, 178]], float)
ROBOT_DOMAINS = {"42", "91", "11"}  # never let the mocap side sit on a robot_remote / robot graph
LATEST_DIR = Path(__file__).resolve().parent.parent / "latest_run"
LATEST = QoSProfile(depth=1)
GRIP_OPEN, GRIP_CLOSED = 0.00483688, -0.0116631  # XEG-32 carriage [m] (rebel_demo / record_dataset.py)


def accel_step(q_prev, v_prev, q_target, vmax, amax, dt, adec):
    """Same as quest_dual_arm_teleop.accel_step: acceleration-limited step toward q_target (speed <= vmax,
    |dv| <= amax dt, slow enough to stop with adec), all joints scaled uniformly. Returns (q, v)."""
    rem = q_target - q_prev
    v_des = rem / dt
    m = float(np.max(np.abs(v_des)))
    if m > vmax:
        v_des = v_des * (vmax / m)
    moving = np.abs(v_des) > 1e-9
    if np.any(moving):
        ratio = float(np.min(np.sqrt(2 * adec * np.abs(rem[moving])) / np.abs(v_des[moving])))
        if ratio < 1.0:
            v_des = v_des * ratio
    dv = v_des - v_prev
    m = float(np.max(np.abs(dv))) / (amax * dt)
    if m > 1.0:
        dv = dv / m
    v = v_prev + dv
    return q_prev + v * dt, v


def leash(q_cmd, q_meas, lead):
    """Keep the command within `lead` [rad] of the measured joints (largest joint; uniform scaling keeps the direction)."""
    d = q_cmd - q_meas
    m = float(np.max(np.abs(d)))
    return q_cmd if m <= lead else q_meas + d * (lead / m)


def make_context(domain, localhost_only):
    """rcl reads ROS_LOCALHOST_ONLY when a context is initialised, so each context gets its own setting."""
    os.environ["ROS_LOCALHOST_ONLY"] = "1" if localhost_only else "0"
    ctx = Context()
    rclpy.init(context=ctx, domain_id=int(domain))
    return ctx


class Follower:
    def __init__(self, args):
        self.a = args
        self.side = args.arm
        self.lock = threading.Lock()
        self.lo = np.deg2rad(LIMITS_DEG[:, 0] + args.limit_margin_deg)
        self.hi = np.deg2rad(LIMITS_DEG[:, 1] - args.limit_margin_deg)
        self.mocap_names = [f"arm_{self.side}_{j}" for j in ARM_JOINTS]
        self.target = None          # newest valid mocap target [rad]
        self.target_t = 0.0
        self.jumped = False         # mocap target jumped: hold until it is back near the arm
        self.q_meas = None
        self.meas_t = 0.0
        self.q_cmd = None
        self.v_cmd = np.zeros(6)
        self.state = "disabled"     # disabled | waiting | follow | glide | hold | home
        self.home_q = np.clip(np.deg2rad(np.array(args.home_deg, float)), self.lo, self.hi)
        self.home_req = None        # why a home move was asked for (picked up by the control tick)
        self.homing = False
        self.home_reached_t = None
        self.enabled = False
        self.reason = "disabled (stdin e / /mocap_follower/enable)"
        self.glide_goal = None
        self.glide_t = 0.0
        self.grip = 0.0             # mocap gripper closedness 0..1 (recorded as the action's gripper value only)
        self.other_q = np.deg2rad(np.array(args.other_arm_deg, float))
        self.stall_ref = None
        self.gap_deg = 0.0
        self.foreign = 0
        self.quit = False
        self.last_status = ""

        robot_domain = os.environ.get("ROS_DOMAIN_ID", "42")
        if str(args.mocap_domain) in ROBOT_DOMAINS or str(args.mocap_domain) == robot_domain:
            raise SystemExit(f"--mocap-domain {args.mocap_domain} is a robot domain; use the mocap PC's domain (12)")
        self.ctx_robot = make_context(robot_domain, True)
        self.ctx_mocap = make_context(args.mocap_domain, False)
        self.rn = Node("mocap_follower", context=self.ctx_robot)
        self.mn = Node("mocap_follower_lan", context=self.ctx_mocap)
        self.log = self.rn.get_logger()

        self.cmd_topic = args.cmd_topic.format(side=self.side)
        self.traj_pub = self.rn.create_publisher(JointTrajectory, self.cmd_topic, 10)
        self.state_topic = args.state_topic.format(side=self.side)
        self.state_pubs = 0
        self.rn.create_subscription(JointState, self.state_topic, self.on_state, LATEST)
        self.rn.create_subscription(Bool, "/mocap_follower/enable", lambda m: self.set_enabled(m.data, "topic"), 1)
        self.mn.create_subscription(JointState, args.mocap_topic, self.on_mocap, LATEST)
        self.fb_pub = None if args.no_feedback else self.mn.create_publisher(JointState, args.feedback_topic, 10)
        self.status_pub = self.mn.create_publisher(String, "/mocap_follower/status", 10)
        # for recording/record_dataset.py --source real (same topics the Quest teleop publishes): the commanded joints
        # + gripper as the action, and the measured / assumed joints of both arms (the undriven arm has no driver)
        self.other = "right" if self.side == "left" else "left"
        self.action_pub = self.rn.create_publisher(JointState, "/quest_teleop/action", 10)
        self.js_pub = self.rn.create_publisher(JointState, "/quest_teleop/joint_states", 10)
        rebel = args.rebel_teleop.rstrip("/")
        self.home_cli = self.mn.create_client(Trigger, f"{rebel}/home")
        if not args.no_home_after_episode:
            self.rn.create_subscription(String, "/recorder/episode_end", lambda m: self.request_home(f"episode {m.data}"), 10)

        self.trace = None
        if args.trace:
            Path(args.trace).parent.mkdir(parents=True, exist_ok=True)
            self.trace_f = open(args.trace, "w", newline="")
            self.trace_f.write(f"# mocap_follower {time.strftime('%Y-%m-%d %H:%M:%S')} argv={' '.join(sys.argv[1:])}\n"
                               "# angles in deg, ages in ms\n")
            self.trace = csv.writer(self.trace_f)
            self.trace.writerow(["t", "state", "sent"] + [f"tgt{i}" for i in range(1, 7)] + [f"cmd{i}" for i in range(1, 7)]
                                + [f"meas{i}" for i in range(1, 7)] + ["tgt_age_ms", "meas_age_ms", "lag_deg", "note"])
        self.t0 = time.monotonic()

    # ---------------------------------------------------------------- inputs
    def on_state(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM_JOINTS):
            return
        q = np.array([msg.position[idx[j]] for j in ARM_JOINTS], float)
        with self.lock:
            self.q_meas, self.meas_t = q, time.monotonic()
        if self.fb_pub is not None:  # mocap side: start pose + its lead limit. The undriven arm is reported at
            # --other-arm-deg too: rebel_demo with rig:=both waits for BOTH arms (its commands for it are ignored)
            other = [f"arm_{self.other}_{j}" for j in ARM_JOINTS]
            self.fb_pub.publish(JointState(header=msg.header, name=self.mocap_names + other,
                                           position=q.tolist() + self.other_q.tolist()))

    def on_mocap(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in self.mocap_names):
            return  # e.g. a message for the other arm only
        q = np.array([msg.position[idx[n]] for n in self.mocap_names], float)
        if not np.all(np.isfinite(q)):
            return
        c = idx.get(f"arm_{self.side}_xeg32_left_carriage_joint")
        grip = None if c is None else float(np.clip((msg.position[c] - GRIP_OPEN) / (GRIP_CLOSED - GRIP_OPEN), 0, 1))
        now = time.monotonic()
        with self.lock:
            if grip is not None:
                self.grip = grip
            if self.target is not None and now - self.target_t < self.a.stale_s:
                step = math.degrees(float(np.max(np.abs(q - self.target))))
                if step > self.a.max_target_jump_deg:
                    self.jumped = True
                    self.reason = f"mocap target jumped {step:.1f} deg in one message"
            self.target, self.target_t = q, now

    def request_home(self, src):
        """Ask the control tick to drive the arm home (only while enabled; ignored while already homing)."""
        with self.lock:
            if not self.enabled:
                self.log.warning(f"[{self.side}] home ({src}) ignored: disabled - type e first")
                return
            if self.homing or self.home_req:
                return
            self.home_req = src

    def call_rebel_home(self, src):
        """rebel_demo ~/home: it resumes from its own last command, so that has to go home with the arm."""
        if not self.home_cli.service_is_ready():
            self.log.warning(f"[{self.side}] home ({src}): {self.home_cli.srv_name} not available - rebel_demo's target "
                             "stays where it was; re-home rebel_demo before resuming")
            return
        self.home_cli.call_async(Trigger.Request()).add_done_callback(
            lambda f: self.log.info(f"[{self.side}] rebel_demo: {f.result().message if f.result() else f.exception()}"))

    def set_enabled(self, on, src):
        with self.lock:
            self.enabled = bool(on)
            if not on:
                self.reason = f"disabled ({src})"
        if on:
            self.log.warning(f"ENABLED ({src})")
        else:
            self.log.warning(f"disabled ({src})")

    # ---------------------------------------------------------------- control tick
    def tick(self):
        now = time.monotonic()
        dt = 1.0 / self.a.rate
        note = ""
        with self.lock:
            tgt, tgt_age = self.target, now - self.target_t
            meas, meas_age = self.q_meas, now - self.meas_t
            jumped, enabled = self.jumped, self.enabled
        if meas is None or meas_age > self.a.state_stale_s or self.state_pubs > 1:
            # no trustworthy feedback: send nothing at all (the controller finishes its last short segment and holds).
            # Two publishers on the state topic interleave their values (e.g. a second driver / viz relay): never follow
            if self.state_pubs > 1:
                why = f"{self.state_pubs} publishers on {self.state_topic} - joint feedback not trustworthy"
            else:
                why = "no /joint_states from the driver" if meas is None else f"joint states stale ({meas_age:.2f} s)"
            if self.homing:  # never resume a home move by itself once the feedback is back
                self.homing = False
                self.log.warning(f"[{self.side}] home stopped: {why}")
            self.set_state("waiting" if meas is None else "hold", why)
            self.q_cmd, self.v_cmd, self.glide_goal = None, np.zeros(6), None
            return self.write(now, False, tgt, meas, tgt_age, meas_age, "")
        if self.q_cmd is None:
            self.q_cmd, self.v_cmd = meas.copy(), np.zeros(6)
        with self.lock:
            req, self.home_req = self.home_req, None
        if req and enabled:
            self.homing, self.home_reached_t, self.glide_goal, self.stall_ref = True, None, None, None
            if self.state != "follow":  # holding / waiting: start from the measured joints
                self.q_cmd, self.v_cmd = meas.copy(), np.zeros(6)
            self.set_state("home", f"moving home ({req})")
            self.call_rebel_home(req)
            note = f"home: {req}"
        if self.homing:
            return self.home_tick(now, dt, tgt, meas, tgt_age, meas_age, enabled, note)

        why = None
        if not enabled:
            why = self.reason
        elif self.foreign:
            why = f"{self.foreign} other publisher(s) on {self.cmd_topic}"
        elif tgt is None or tgt_age > self.a.stale_s:
            why = "no mocap commands" if tgt is None else f"mocap stream stale ({tgt_age:.2f} s)"
        elif jumped:
            if math.degrees(float(np.max(np.abs(tgt - meas)))) <= self.a.engage_deg:
                with self.lock:
                    self.jumped = False
                note = "mocap target back near the arm"
            else:
                why = self.reason
        elif self.state != "follow" and math.degrees(float(np.max(np.abs(tgt - meas)))) > self.a.engage_deg:
            # constant text (set_state logs every change); the current gap goes into the 5 s status line
            self.gap_deg = math.degrees(float(np.max(np.abs(tgt - meas))))
            why = (f"waiting for the mocap target to come within {self.a.engage_deg:g} deg of the arm "
                   "(it is never moved to a far target; re-home / restart rebel_demo to align)")

        sent = False
        if why is None:
            if self.state != "follow":
                note = note or "following"
                self.stall_ref = None
                self.glide_goal = None
            self.set_state("follow", "following")
            goal = np.clip(tgt, self.lo, self.hi)
            q, v = accel_step(self.q_cmd, self.v_cmd, goal, self.a.max_joint_vel, self.a.max_joint_acc, dt,
                              self.a.max_joint_dec)
            q = leash(q, meas, math.radians(self.a.max_cmd_lead_deg))
            self.v_cmd, self.q_cmd = (q - self.q_cmd) / dt, q
            if self.stalled(meas, now):  # do not glide: the arm is not following anything we send
                self.glide_goal = None
                self.set_state("hold", "arm does not follow its command (motors off, e-stop, blocked?)")
                self.disable_after_fault()
                self.q_cmd, self.v_cmd = meas.copy(), np.zeros(6)
            else:
                sent = self.send(self.q_cmd)
        else:
            sent = self.stop(why, meas, dt)
        self.publish_record(meas)
        self.write(now, sent, tgt, meas, tgt_age, meas_age, note)

    def home_tick(self, now, dt, tgt, meas, tgt_age, meas_age, enabled, note):
        """Joint-space move to --home-deg, the mocap target ignored; stopped by `d`, another command publisher or an
        arm that does not follow. Done when the command is home and the arm has settled there."""
        why = self.reason if not enabled else (f"{self.foreign} other publisher(s) on {self.cmd_topic}" if self.foreign
                                               else None)
        if why is not None:
            self.homing = False
            self.log.warning(f"[{self.side}] home stopped: {why}")
            sent = self.stop(why, meas, dt)
        else:
            q, v = accel_step(self.q_cmd, self.v_cmd, self.home_q, math.radians(self.a.home_vel_deg),
                              self.a.max_joint_acc, dt, self.a.max_joint_dec)
            q = leash(q, meas, math.radians(self.a.max_cmd_lead_deg))
            self.v_cmd, self.q_cmd = (q - self.q_cmd) / dt, q
            sent = False
            if self.stalled(meas, now):
                self.homing = False
                self.set_state("hold", "arm does not follow its command (motors off, e-stop, blocked?)")
                self.disable_after_fault()
                self.q_cmd, self.v_cmd = meas.copy(), np.zeros(6)
            else:
                sent = self.send(self.q_cmd)
                if float(np.max(np.abs(self.q_cmd - self.home_q))) < 1e-4:
                    self.home_reached_t = self.home_reached_t or now
                    off = math.degrees(float(np.max(np.abs(meas - self.home_q))))
                    if off < 1.0 or now - self.home_reached_t > 2.0:  # the controller settles ~0.7-1 deg short
                        self.homing = False
                        self.v_cmd = np.zeros(6)
                        self.set_state("hold", f"at home ({off:.1f} deg); follows again once the mocap target is "
                                               "back on the arm (SPACE in teleop_keyboard)")
                        note = note or "at home"
        self.publish_record(meas)
        self.write(now, sent, tgt, meas, tgt_age, meas_age, note)

    def publish_record(self, meas):
        """Action = what this node commands (the measured joints while it sends nothing) + mocap gripper; the
        undriven arm: --other-arm-deg in both. Joint states: measured driven arm + assumed undriven arm."""
        q = self.q_cmd if self.q_cmd is not None else meas
        if q is None:
            return
        stamp = self.rn.get_clock().now().to_msg()
        arms = {self.side: (q, self.grip), self.other: (self.other_q, 0.0)}
        names, pos = [], []
        for s in ("left", "right"):
            names += [f"arm_{s}_{j}" for j in ARM_JOINTS] + [f"arm_{s}_gripper"]
            pos += [float(v) for v in arms[s][0]] + [float(arms[s][1])]
        msg = JointState(name=names, position=pos)
        msg.header.stamp = stamp
        self.action_pub.publish(msg)
        js = {self.side: meas, self.other: self.other_q}
        msg = JointState(name=[f"arm_{s}_{j}" for s in ("left", "right") for j in ARM_JOINTS],
                         position=[float(v) for s in ("left", "right") for v in js[s]])
        msg.header.stamp = stamp
        self.js_pub.publish(msg)

    def stop(self, why, meas, dt):
        """Glide: keep streaming an acceleration-limited command until it stands still, then send nothing."""
        if self.state in ("follow", "home"):
            v = self.v_cmd
            dist = v * np.abs(v) / (2 * self.a.glide_dec)
            m = float(np.max(np.abs(dist)))
            if m > math.radians(20):
                dist = dist * (math.radians(20) / m)
            self.glide_goal = np.clip(self.q_cmd + dist, self.lo, self.hi)
            self.glide_t = time.monotonic()
            self.set_state("glide", why)
        if self.state == "glide" and self.glide_goal is not None:
            q, v = accel_step(self.q_cmd, self.v_cmd, self.glide_goal, self.a.max_joint_vel, self.a.glide_dec,
                              dt, self.a.glide_dec)
            self.q_cmd, self.v_cmd = leash(q, meas, math.radians(self.a.max_cmd_lead_deg)), v
            done = (float(np.max(np.abs(v))) < 1e-3 or float(np.max(np.abs(self.q_cmd - self.glide_goal))) < 1e-3
                    or time.monotonic() - self.glide_t > self.a.glide_max_s)
            if done:  # stopped, reached, or out of time: send nothing from the next tick on
                self.glide_goal = None
            return self.send(self.q_cmd)
        self.set_state("disabled" if why.startswith("disabled") else "hold", why)
        self.q_cmd, self.v_cmd = meas.copy(), np.zeros(6)  # re-start from the measured joints
        return False

    def stalled(self, meas, now):
        ref = self.stall_ref
        if ref is None or math.degrees(float(np.max(np.abs(meas - ref[1])))) > 0.5:
            self.stall_ref = (now, meas.copy())
            return False
        lag = math.degrees(float(np.max(np.abs(self.q_cmd - meas))))
        return lag >= 3.0 and now - ref[0] > self.a.stall_s

    def disable_after_fault(self):
        with self.lock:
            self.enabled = False
            self.reason = "disabled after a fault (arm did not follow); check the arm, then `e`"

    def send(self, q):
        if self.a.dry_run:
            return False
        la = self.a.lookahead
        pt = JointTrajectoryPoint(positions=[float(v) for v in q],
                                  time_from_start=Duration(sec=int(la), nanosec=int((la % 1) * 1e9)))
        self.traj_pub.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[pt]))
        return True

    def set_state(self, state, why):
        if (state, why) != (self.state, self.last_status):
            if state in ("hold", "glide") and "disabled" not in why:  # two call sites: rclpy forbids one site
                self.log.warning(f"[{self.side}] {state}: {why}")       # changing its severity
            else:
                self.log.info(f"[{self.side}] {state}: {why}")
        self.state, self.last_status = state, why

    def check_foreign(self):
        self.foreign = max(0, self.rn.count_publishers(self.cmd_topic) - (0 if self.a.dry_run else 1))
        self.state_pubs = self.rn.count_publishers(self.state_topic)

    def write(self, now, sent, tgt, meas, tgt_age, meas_age, note):
        if self.trace is None:
            return
        deg = lambda q: [f"{v:.3f}" for v in np.rad2deg(q)] if q is not None else [""] * 6  # noqa: E731
        lag = f"{math.degrees(float(np.max(np.abs(self.q_cmd - meas)))):.2f}" if self.q_cmd is not None and meas is not None else ""
        self.trace.writerow([f"{now - self.t0:.4f}", self.state, int(sent)] + deg(tgt) + deg(self.q_cmd) + deg(meas)
                            + [f"{tgt_age * 1e3:.0f}" if tgt is not None else "", f"{meas_age * 1e3:.0f}" if meas is not None else "",
                               lag, note])

    def publish_status(self):
        with self.lock:
            tgt_age = time.monotonic() - self.target_t if self.target is not None else None
        gap = f" (target {self.gap_deg:.1f} deg away)" if self.last_status.startswith("waiting for") else ""
        txt = (f"{self.side} {self.state}: {self.last_status}{gap} | enabled={self.enabled} dry_run={self.a.dry_run} "
               f"mocap_age={'-' if tgt_age is None else f'{tgt_age:.2f}s'}")
        self.status_pub.publish(String(data=txt))
        self.log.info(txt, throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- run
    def run(self):
        if self.a.dry_run:
            self.log.warning(f"[{self.side}] DRY RUN: nothing is sent to the arm")
        ex_r, ex_m = SingleThreadedExecutor(context=self.ctx_robot), SingleThreadedExecutor(context=self.ctx_mocap)
        ex_r.add_node(self.rn)
        ex_m.add_node(self.mn)
        threads = [threading.Thread(target=ex.spin, daemon=True) for ex in (ex_r, ex_m)]
        for t in threads:
            t.start()
        time.sleep(1.0)  # discovery
        self.check_foreign()
        refuse = None  # refusals also go through the finally below (threads stopped before exit: no abort)
        if self.foreign and not self.a.dry_run:
            refuse = (f"refusing to start: {self.foreign} other publisher(s) on {self.cmd_topic} "
                      "(Quest teleop, RViz pretest, a test?) - stop them first")
        elif self.state_pubs > 1:
            refuse = (f"refusing to start: {self.state_pubs} publishers on {self.state_topic} "
                      "(a second driver?) - the joint feedback would be mixed")
        self.quit = refuse is not None
        if not refuse:
            self.log.info(f"[{self.side}] mocap domain {self.a.mocap_domain}: {self.a.mocap_topic} -> {self.cmd_topic}; "
                          f"feedback {'off' if self.fb_pub is None else self.a.feedback_topic}; limits "
                          f"{math.degrees(self.a.max_joint_vel):.0f} deg/s, lead {self.a.max_cmd_lead_deg:.0f} deg. "
                          f"home {np.round(np.degrees(self.home_q), 1).tolist()} deg at "
                          f"{self.a.home_vel_deg:g} deg/s on: h"
                          f"{'' if self.a.no_home_after_episode else ', episode end'}"
                          ". "
                          "DISABLED until stdin `e` (d = disable, h = home, q = quit)")
            if self.a.enable:
                self.set_enabled(True, "--enable")
            threading.Thread(target=self.stdin_loop, daemon=True).start()
        period, next_t, last_chk, last_status = 1.0 / self.a.rate, time.monotonic(), 0.0, 0.0
        try:
            while not self.quit:
                now = time.monotonic()
                if now - last_chk > 0.5:
                    self.check_foreign()
                    last_chk = now
                self.tick()
                if now - last_status > 1.0:
                    self.publish_status()
                    last_status = now
                next_t += period
                time.sleep(max(0.0, next_t - time.monotonic()))
                if next_t < time.monotonic() - 1.0:
                    next_t = time.monotonic()
            # quit: glide to a stop before exiting
            if not refuse:
                self.set_enabled(False, "quit")
            end = time.monotonic() + 3.0
            while self.state == "glide" and time.monotonic() < end:
                self.tick()
                time.sleep(period)
        finally:
            if self.trace is not None:
                self.trace_f.close()
            for ex in (ex_r, ex_m):
                ex.shutdown(timeout_sec=1.0)
            for t in threads:  # spin threads must end before the nodes go (else: terminate called, abort)
                t.join(timeout=2.0)
            for node, ctx in ((self.rn, self.ctx_robot), (self.mn, self.ctx_mocap)):
                node.destroy_node()
                rclpy.shutdown(context=ctx)
        if refuse:
            raise SystemExit(refuse)

    def stdin_loop(self):
        for line in sys.stdin:
            c = line.strip().lower()
            if c in ("e", "enable"):
                self.set_enabled(True, "stdin")
            elif c in ("d", "disable"):
                self.set_enabled(False, "stdin")
            elif c in ("h", "home"):
                self.request_home("stdin")
            elif c in ("q", "quit"):
                self.quit = True
                return


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", choices=["left", "right"], required=True)
    p.add_argument("--mocap-domain", default=os.environ.get("MOCAP_ROS_DOMAIN_ID", "12"))
    p.add_argument("--mocap-topic", default="/dual_arm/isaac_joint_commands")
    p.add_argument("--feedback-topic", default="/joint_states", help="measured joints for rebel_demo (mocap domain)")
    p.add_argument("--no-feedback", action="store_true")
    p.add_argument("--state-topic", default="/{side}/joint_states")
    p.add_argument("--cmd-topic", default="/{side}/rebel_arm_trajectory_controller/joint_trajectory")
    p.add_argument("--dry-run", action="store_true", help="compute everything, send nothing to the arm")
    p.add_argument("--enable", action="store_true", help="start enabled (default: wait for stdin `e`)")
    p.add_argument("--rate", type=float, default=30.0)
    p.add_argument("--lookahead", type=float, default=0.15, help="time_from_start of each single-point trajectory [s]")
    p.add_argument("--max-joint-vel", type=float, default=0.26, help="[rad/s] (0.26 = the launcher's 'precise')")
    p.add_argument("--max-joint-acc", type=float, default=0.52, help="[rad/s^2]")
    p.add_argument("--max-joint-dec", type=float, default=0.3, help="braking deceleration toward a target [rad/s^2]")
    p.add_argument("--glide-dec", type=float, default=0.4, help="deceleration when stopping on a hold [rad/s^2]")
    p.add_argument("--glide-max-s", type=float, default=3.0, help="a stop glide sends for at most this long")
    p.add_argument("--other-arm-deg", type=float, nargs=6, default=[0.0] * 6, metavar="J",
                   help="pose of the arm this node does not drive, for the recorder (like the Quest teleop's)")
    p.add_argument("--max-cmd-lead-deg", type=float, default=10.0, help="command at most this far ahead of the arm")
    p.add_argument("--limit-margin-deg", type=float, default=3.0, help="stay this far inside the joint limits")
    p.add_argument("--stale-s", type=float, default=0.25, help="hold when the newest mocap command is older")
    p.add_argument("--state-stale-s", type=float, default=0.5, help="send nothing when the joint states are older")
    p.add_argument("--max-target-jump-deg", type=float, default=12.0, help="hold when one mocap message jumps more")
    p.add_argument("--engage-deg", type=float, default=3.0,
                   help="start / resume following only when the mocap target is this close to the arm: the node "
                        "never moves the arm to a far (paused / stale) target by itself; mocap must bring it there")
    p.add_argument("--stall-s", type=float, default=1.0, help="hold when the arm does not move this long while behind")
    p.add_argument("--home-deg", type=float, nargs=6, default=[0.0, 0.0, 60.0, 0.0, 50.0, 0.0], metavar="J",
                   help="home pose: use rebel_demo's home_deg, or it will not follow again after a home move")
    p.add_argument("--home-vel-deg", type=float, default=10.0, help="joint speed of the home move [deg/s]")
    p.add_argument("--rebel-teleop", default="/rebel_teleop", help="rebel_demo teleop node whose ~/home is called")
    p.add_argument("--no-home-after-episode", action="store_true",
                   help="do not home when the recorder saves / discards an episode (/recorder/episode_end)")
    p.add_argument("--trace", default=str(LATEST_DIR / "mocap_follower_trace.csv"), help="'' = no trace file")
    f = Follower(p.parse_args())
    signal.signal(signal.SIGINT, lambda *_: setattr(f, "quit", True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(f, "quit", True))
    f.run()


if __name__ == "__main__":
    main()
