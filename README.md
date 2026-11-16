# robot_remote: mocap recording on the igus ReBeL right arm

OptiTrack mocap drives the physical right igus ReBeL arm. Every demonstration is saved as a LeRobot dataset, and each
frame is labelled with the box flap being opened.

**Before you start:**
- The mocap side (OptiTrack + rebel_demo) is running and publishing on ROS domain 12.
- The robot network is up: `nmcli con up robot-eno1`.
- Someone stands at the e-stop.

## Quickstart: five terminals

Open them in this order.

### Terminal 1: camera names (after plugging cameras in)

```bash
cd ~/robot_remote-main && ./recording/run_camera_setup.sh
```

Pick `head` and `wrist_right` under the pictures, press **Save**, then close the window. A camera can be open in only one
program, so the window must be closed before you record.

### Terminal 2: arm driver

```bash
cd ~/robot_remote-main
source /opt/ros/humble/setup.bash && source igus_ws_jtc/install/setup.bash
ros2 launch real_robot/dual_arm_driver.launch.py arms:=right
```

Wait for `success=True`: the motors are enabled and the arm holds its pose.

### Terminal 3: mocap follower

```bash
cd ~/robot_remote-main
./real_robot/run_mocap_follower.sh --arm right --mocap-domain 12
```

Type `e` + Enter to enable it. The arm follows the mocap once the mocap target is within 3° of the arm; `h` + Enter
sends the arm home.

### Terminal 4: recorder

```bash
cd ~/robot_remote-main
ISAAC_SIM=$HOME/isaacsim ./recording/run_record.sh --source real --root /media/ai/aipc/Karel/Mocap/Dataset --name new_mocap_allflaps3 --task "open the box flaps" --cameras head wrist_right --flaps-sequence
```

Wait for `[rec] controls: …`. A new `--name` creates a new dataset; an existing name adds episodes to it.

### Terminal 5: check the recordings

```bash
cd ~/robot_remote-main && ./launcher/run_replay_gui.sh /media/ai/aipc/Karel/Mocap/Dataset/new_mocap_allflaps1
```

Pick an episode to see its flap order and times. **View** plays the cameras and joint curves with the current flap
shown.

## Record an episode (keys in terminal 4)

1. **Enter**: start recording. The episode starts on flap 1.
2. **`n` + Enter**: this flap is finished, go to the next one (pause the arm first). Four flaps = three `n`.
3. **Enter**: save. The recorder prints `flaps: 4 of 4 reached` and `■ saved`, and the follower brings the arm home.

- **`d` + Enter** discards the open episode.
- **`q` + Enter** quits. Always quit this way; closing the terminal can leave the last episodes unreadable.

The flap order is in `config/flaps_sequence.yaml`; edit it before starting the recorder. Use one order per dataset name.

## Stop

Recorder `q` → follower `q` → driver Ctrl+C.

## More

- [`docs/mocap_quickstart.md`](docs/mocap_quickstart.md): details, the mocap-side terminals, camera settings,
  troubleshooting.
- Install on a new machine: `./setup/install.sh`.
- Quest teleop / Isaac Sim (the former README): [`docs/quest_teleop.md`](docs/quest_teleop.md).
- Shared notes for coding agents: [`AGENTS.md`](AGENTS.md). Third-party material: [`THIRD_PARTY.md`](THIRD_PARTY.md).
