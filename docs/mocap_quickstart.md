# Mocap quickstart: record box-flap demos on the real right arm

Your hand, tracked by OptiTrack, drives the physical right igus ReBeL arm. Every demonstration is saved as a LeRobot
dataset in which each frame is labelled with the flap being opened ("open the front flap", ...).

```
OptiTrack (Motive) ──► rebel_demo IK (ROS domain 12) ──► mocap_follower ──► igus driver (domain 42, this PC only) ──► right arm
                                                               │ commanded joints            joint states │
                                                               ▼                                          ▼
                                     recorder + head and wrist_right USB cameras ──► LeRobot dataset on /media/ai/aipc/Karel/Mocap/Dataset
```

All commands run on the lab PC (`ai-iece`) from `~/robot_remote-main`, on branch `main`. The short version with the
five robot_remote terminals is the [README](../README.md); this page adds the mocap-side terminals, details and
troubleshooting.

## Before you start

- **Someone stays at the e-stop** for the whole session. The follower has no collision check: table, box and stand
  are not modelled.
- **Robot network:** `nmcli con up robot-eno1`, then `ping -c 3 192.168.3.12` (right arm) must answer.
- **OptiTrack:** Motive is streaming the hand rigid bodies.
- **Cameras:** the head camera (Orbbec Gemini 336) in a **USB 3** port of the PC; the right-wrist camera in its usual
  port (it is found by port; see step 1 after re-plugging).
- **Disk:** datasets go to `/media/ai/aipc/Karel/Mocap/Dataset`. The home folder has a 250 GB quota that is
  almost full; when it fills up, ROS and the recorder stop with `Disk quota exceeded`.
- `$R` below is `~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh` (already set in `~/.bashrc`; in another
  shell: `R=~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh`).

## 1. Cameras (only after re-plugging them, or if a picture looks wrong)

```bash
cd ~/robot_remote-main && ./recording/run_camera_setup.sh
```

The window shows every connected camera live with a name menu under it. Pick **head** for the camera above the box,
**wrist_right** for the one on the right arm, **(not used)** for the rest. Picking a name another camera already has
swaps the two. **Save** writes `config/real_cameras.json` (the old file is kept as `real_cameras.json.bak`). **Close the
window before recording**: a camera can be open in only one program.

Optional check, both cameras together for 20 s:

```bash
./recording/run_cameras.sh --test 20 --cameras head wrist_right
```

Both cameras should report at least 30 fps. Always pass `--cameras`: without it the test also tries the configured
cameras that are not plugged in.

The camera settings in that file are applied each time a camera opens: 60 Hz anti-flicker on both cameras, and the head
exposure fixed at 8.3 ms so the room lights don't show as dark bands.

## 2. Start: one terminal each, in this order

**Terminal 1: OptiTrack driver**

```bash
$R driver
```

When it prints `Configured!`, run this in another terminal:

```bash
$R activate
```

**Terminal 2: arm driver.** Release the e-stop first: the motors are enabled about 6 s after the start.

```bash
cd ~/robot_remote-main
source /opt/ros/humble/setup.bash && source igus_ws_jtc/install/setup.bash
ros2 launch real_robot/dual_arm_driver.launch.py arms:=right
```

Ready when you see `gains tuned, jog_velocity_scale 1.65` and then `success=True`. The arm now holds its pose.
`igus_ws_jtc` is this PC's build of the newer igus driver (Igus_rebel 287b0186). It is not in git. On a machine
installed from this branch with `setup/install.sh`, `igus_ws` already holds that driver: source
`igus_ws/install/setup.bash` instead.

**Terminal 3: rebel_demo** (mocap → joint targets). Use a **fresh** terminal with nothing from `robot_remote` sourced.

```bash
$R real rig:=right
```

**Terminal 4: keyboard.** Keep this window focused during the demos: **SPACE** engages or pauses the mocap.

```bash
$R keyboard
```

**Terminal 5: follower** (mocap targets → arm driver)

```bash
cd ~/robot_remote-main && ./real_robot/run_mocap_follower.sh --arm right --mocap-domain 12
```

It starts **disabled**. Type `e` + Enter to enable it.

- It starts following only once the mocap target is within 3° of the arm. It never moves the arm to a far target.
  Its status line then says `waiting for the mocap target to come within 3 deg of the arm (target X deg away)`.
- To line them up, type `h` + Enter: the arm and rebel_demo's target both go home, (0, 0, 60, 0, 50, 0)°, at 10°/s.

**Terminal 6: recorder**

```bash
cd ~/robot_remote-main
ISAAC_SIM=$HOME/isaacsim ./recording/run_record.sh --source real --root /media/ai/aipc/Karel/Mocap/Dataset \
  --name new_mocap_allflaps3 --task "open the box flaps" --cameras head wrist_right --flaps-sequence
```

Ready when it prints `[rec] controls: …` and `[rec] flap order (…): …`. Check that the flap order is the one you are
going to demonstrate.

## 3. Record an episode

1. Put your hand in place and press **SPACE** (terminal 4). The arm follows your hand.
2. Press **Enter** in the recorder (terminal 6). It prints `[rec] ● RECORDING episode N`. You are on flap 1; no key is
   needed. The status line every second shows `[flap 1/4: open the front flap]`.
3. Open the flap. When it is open, press **SPACE** to pause the arm, then type **`n`** + Enter in the recorder:
   `… finished -> now flap 2/4: …`. Press **SPACE** to continue.
4. Do the same after flaps 2 and 3. That is three `n` for four flaps; the last flap needs no `n`.
5. Press **Enter** in the recorder to save. It prints `[rec] flaps: 4 of 4 reached …` and then
   `[rec] ■ saved. total episodes: N`. The follower brings the arm home by itself, and rebel_demo's target goes home
   with it.
6. Wait for `■ saved` before you start the next episode.

**Mistakes:**

- **`d` + Enter** discards the open episode (`✕ discarded`). Nothing is saved, and the arm goes home.
- **A missed `n`** saves fewer than 4 flaps (`flaps: 3 of 4 reached … <- fewer than all flaps: a missed 'n'?`), and
  the labels of every later flap are wrong. Discard (`d`) and redo the episode. An extra `n` on the last flap is ignored.
- Very short episodes are discarded automatically (`episode too short`).
- **`stale: wrist_right`** in the recorder's status line: the wrist camera stopped sending, and the video freezes.
  Discard (`d`) and see *Troubleshooting*.

**Keys**

| Window | Key | Effect |
| --- | --- | --- |
| Keyboard (4) | SPACE | Engage / pause the mocap |
| Follower (5) | `e` | Enable (start of session) |
| | `d` | Disable: the arm glides to a stop and holds |
| | `h` | Arm and rebel_demo's target go home |
| | `q` | Quit (the arm glides to a stop first) |
| Recorder (6) | Enter | Start / save an episode |
| | `n` | This flap is finished: next flap |
| | `d` | Discard the open episode |
| | `q` | Quit: closes the dataset files. **Always quit this way.** |

## 4. Flap order and dataset names

- The order comes from `config/flaps_sequence.yaml`, one sentence per step. Edit it **before** starting the recorder;
  the recorder prints the order when it starts. Each sentence becomes the LeRobot `task` of the frames recorded while
  that flap is open. `--task` is only used when `--flaps-sequence` is left out.
- **Keep one flap order per dataset.** When you change the order, change `--name` too. Existing datasets:

  | Dataset | Flap order |
  | --- | --- |
  | `new_mocap_allflaps1` | back → front → right → left |
  | `new_mocap_allflaps2` | front → back → right → left |
  | `new_mocap_allflaps3` | front → back → left → right |
  | `new_mocap_allflaps4` | back → front → left → right |

- A new `--name` creates a dataset. An existing name adds episodes to it, but only with the same cameras and fps;
  otherwise the recorder refuses. It also warns when a dataset with flap labels would be mixed with unlabelled
  episodes.
- In the dataset, every frame's LeRobot `task` is its flap sentence: `LeRobotDataset(...)[i]["task"]` gives it, and
  `meta/tasks.parquet` lists the sentences. A flap starts at the moment `n` was typed, so the pause before it belongs to
  the previous flap.
- Where each flap starts is in `<dataset>/meta/subtask_marks.jsonl`, one line per saved episode:
  `{"episode_index": 0, "subtasks": [...], "start_frames": [0, 702, …]}`. A discarded episode leaves nothing. The
  `tasks` list LeRobot keeps per episode (`meta/episodes`) is an unordered set: take the order from the per-frame task
  or from this file.
- There is no length limit. `--max-episode-s N` would save automatically after N seconds.

## 5. Check the recordings

```bash
cd ~/robot_remote-main && ./launcher/run_replay_gui.sh /media/ai/aipc/Karel/Mocap/Dataset/new_mocap_allflaps1
```

- The episode list shows each episode's length and flap order, e.g. `open front → back → left → right`. Fewer than four
  means a missed `n`.
- Selecting an episode shows each flap with its time span below the list.
- **View** opens the LeRobot viewer (Rerun): both camera videos, the joint curves, and a `task` panel that changes with
  the playback.

## 6. Stop

1. Recorder: `q` + Enter, then wait until it exits. Don't close the terminal or kill the recorder: the last episodes
   would be unreadable.
2. Follower: `q` + Enter.
3. Keyboard: `q`. Then rebel_demo and the OptiTrack driver: Ctrl+C.
4. Arm driver last: Ctrl+C.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| Recorder: `No module named 'rclpy'` | `ISAAC_SIM=$HOME/isaacsim` is missing; it must be on the same line or `export`ed. Don't source `/opt/ros` in the recorder terminal. |
| Driver: `gains:=tuned needs jog_velocity_scale …` | The old driver is sourced. Source `igus_ws_jtc/install/setup.bash`, not `igus_ws`. (`igus_ws_jtc` = Igus_rebel 287b0186 with the `tuned` gains; adding `gains:=standard` to the launch gives the old controller behaviour on the new driver.) |
| Driver fails at start with a library error (e.g. `libssl.so.3`) | Conda's libraries got in the way: `conda deactivate`, then source and launch again. |
| Driver never prints `success=True`, or the arm does not hold | Check the e-stop and `ping 192.168.3.12`. Re-enable from a terminal sourced like terminal 2, plus `source ros_env.sh`: `ros2 service call /right/igus_rebel/reset_and_enable std_srvs/srv/Trigger`. After a network drop, restart the driver. |
| rebel_demo: `dual_arm_rig_v2.urdf.xacro: No such file` | That terminal also sourced robot_remote's ROS workspace. Use a fresh terminal. |
| Follower stays at `waiting for the mocap target to come within 3 deg …` | rebel_demo's target is far from the arm. Type `h` in the follower (or restart rebel_demo), then SPACE. |
| Follower: `arm does not follow its command (motors off, e-stop, blocked?)` and disables itself | The e-stop was pressed, or the motor stopped, e.g. the tool hit the table (J5 overcurrent). Clear the cause, re-enable the motors (row above), then `e`. |
| Recorder status: `stale: wrist_right` | The wrist camera dropped out: USB bandwidth, usually because the head camera is on a USB 2 port. `d`, move the head camera to a USB 3 port, restart the recorder. Check the head camera's speed (5000 = USB 3): `for d in /sys/bus/usb/devices/*; do [ "$(cat $d/idVendor 2>/dev/null)" = 2bc5 ] && echo "$(cat $d/product) $(cat $d/speed)"; done` |
| A camera is missing or has the wrong name | Close the recorder, then run `./recording/run_camera_setup.sh` (step 1). |
| Dark horizontal bands in the head video | Flicker of the room lights (120 Hz on 60 Hz mains). In `config/real_cameras.json` keep `power_line_frequency` 2 and an exposure that is a multiple of 8.33 ms: `auto_exposure` 1 (manual) with `exposure_time_absolute` 83, 167 or 250 (units of 0.1 ms). `./recording/run_cameras.sh --controls --cameras head wrist_right` lists every setting with its range. |
| In a still scene every second video frame looks identical | Normal: the video compression drops the sensor noise of every other frame. No picture is lost. |
| `Disk quota exceeded` | The home folder is full. Free space in `/home/user1`; keep datasets on `/media/ai/aipc`. |
| Need the follower's log of a session | `latest_run/mocap_follower_trace.csv` is overwritten on every follower start; copy it before restarting. |
