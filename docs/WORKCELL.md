# Using the workcell

How to run the one-arm workcell, in simulation and on the real UR5e, and how
to move it. This guide covers operating it. Why it is built this way is in
the header of
[`workcell.urdf.xacro`](../ros2_ws/src/bartender_description/urdf/workcell.urdf.xacro).

## What it is

One UR5e with a Robotiq 2F-85 gripper, on a 1.40 x 0.70 m table. The robot's
base is 0.35 m in from one end and 0.35 m in from the long side:

```
   y
   ^
0.70 +------------------------------------------+
     |                                          |
0.35 |   (UR5e) -> +x                           |
     |                                          |
   0 +------------------------------------------+--> x
     0   0.35                                 1.40
```

The table is part of the robot model, so MoveIt will not plan a move that
hits it, in sim or on the real arm. It does **not** know about anything
standing on the table (bottles, boxes, your hand).

The two-armed bar simulation is separate and unchanged. Nothing here affects
it.

## Ways to run it

Each command starts everything needed: the arm, its controllers and MoveIt.
Run one at a time.

| Mode | Command | Needs |
|---|---|---|
| Simulation | `ros2 launch bartender_bringup workcell_sim.launch.py` | Nothing. Gazebo opens. Allow ~20 s. |
| Dry run, no robot | `ros2 launch bartender_bringup workcell_real.launch.py use_fake_hardware:=true` | `ur_robot_driver` installed. No Gazebo; the arm is a mock that goes wherever it is told. |
| Real robot | `ros2 launch bartender_bringup workcell_real.launch.py` | Everything in "One-time setup" below. |
| **Real robot + Gazebo twin** | `ros2 launch bartender_bringup workcell_twin.launch.py` | The same as the real robot. Gazebo also opens and its arm copies the real one. |
| Twin, dry run | `ros2 launch bartender_bringup workcell_twin.launch.py use_fake_hardware:=true` | `ur_robot_driver` installed. Gazebo copies the mock arm. |

The robot's address, `10.42.0.100`, is the default; pass `robot_ip:=...` only
if it changes.

Use the dry run to check the software before the robot is involved. It runs
the same controllers and MoveIt as the real launch.

### The twin

`workcell_twin.launch.py` is the real launch plus the workcell simulation,
side by side. The **real robot leads and Gazebo follows**: a small node,
`twin_mirror`, copies the real arm's joint angles and gripper onto the
simulated arm 50 times a second. The twin shows every move, whether it came
from our teach pendant, MoveIt, or someone jogging on the UR pendant.

It only goes one way. Nothing done in Gazebo moves the real robot, and the
twin takes no goals of its own. You drive the robot exactly as without the
twin (same teach pendant, same commands), and Gazebo shows what it did.

The twin's ROS names are all under `/twin` (`/twin/joint_states`,
`/twin/controller_manager`, ...), so they never mix with the real robot's.
If the real robot stops publishing, the twin holds its last pose and
`twin_mirror` logs a warning.

Build first, and source the workspace in every terminal:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## One-time setup for the real robot

### 1. Install the UR driver

```bash
sudo apt install ros-humble-ur-robot-driver
```

Or rebuild the Docker image with `docker/run_docker.sh`; the Dockerfile
already includes it. The container uses host networking, so the robot is
reachable from inside it with no extra configuration.

### 2. Put the PC and the robot on the same network

Connect the Ethernet cable from the robot's control box to this PC's wired
port, `eno1`.

| | Address | Netmask |
|---|---|---|
| Robot | `10.42.0.100` | `255.255.255.0` |
| This PC (`eno1`) | `10.42.0.67` | `255.255.255.0` |

- **Robot**: on the UR pendant, Settings > System > Network: Static Address,
  IP `10.42.0.100`, netmask `255.255.255.0`. Leave the gateway empty.
- **PC**: `eno1` already has `10.42.0.67` on this machine (check with
  `ip -br addr show eno1`). On another PC, give it that address with no
  gateway, so the WiFi keeps the internet route:

  ```bash
  sudo nmcli con add type ethernet ifname eno1 con-name ur5e \
    ipv4.method manual ipv4.addresses 10.42.0.67/24 ipv6.method disabled
  sudo nmcli con up ur5e
  ping 10.42.0.100
  ```

Continue only once `ping` gets replies.

### 3. Install the External Control URCap on the pendant

This is the program on the robot that hands control to ROS. The file ships
with the driver:

```bash
ls $(ros2 pkg prefix ur_robot_driver)/share/ur_robot_driver/resources/externalcontrol-*.urcap
```

1. Copy it to a USB stick and plug that into the pendant.
2. Settings > System > URCaps > **+**, select the file, restart when asked.
3. Installation > URCaps > External Control: set **Host IP** to the PC's
   address, `10.42.0.67`, and **port 50002**.
4. Make a new program containing just one node, **External Control**. Save
   it (for example as `ros.urp`).

### 4. Open the firewall, if one is on

The robot connects back to the PC on TCP ports 50001-50004.

```bash
sudo ufw status          # if "inactive", skip this step
sudo ufw allow from 10.42.0.100 to any port 50001:50004 proto tcp
```

### 5. Check the three unmeasured settings

These are launch arguments. Their defaults are guesses:

| Argument | Default | What to do |
|---|---|---|
| `arm_yaw` | `-1.5708` | Set from the real cell (the base is turned 90° clockwise, seen from above). |
| `table_height` | `0.75` | Measure the table from floor to top, in metres. |
| `gripper_fake_hardware` | `true` | The gripper is simulated: it says it moved and does nothing. Leave it on until you decide how the gripper is wired. |

Pass any of them on the launch line, e.g. `table_height:=0.72`.

## Starting the real robot, every time

**Our robot is in Remote Control mode** (the dashboard says
`is in remote control: true`). In that mode the UR pendant cannot start a
program by hand, so use headless mode: the driver sends its control script
over the network and no External Control program is needed at all.

```bash
ros2 launch bartender_bringup workcell_twin.launch.py    # headless_mode:=true is the default
```

Then, from our teach pendant: `robot on` if it is not powered, then check
`robot`. The program should say `running`. After a stop, `robot resend`
starts the script again (not `robot play`, which would play whatever program
happens to be loaded on the UR controller).

The steps below are for Local mode, with the External Control program
(launch with `headless_mode:=false`):

1. On the pendant: power on and release the brakes (the red button at the
   bottom left, then ON, then START).
2. On the PC:

   ```bash
   ros2 launch bartender_bringup workcell_twin.launch.py    # with the Gazebo twin
   # or
   ros2 launch bartender_bringup workcell_real.launch.py    # without it
   ```

3. On the pendant: load `ros.urp` and press **Play**.
4. Wait for this line in the PC's terminal:

   ```
   Robot connected to reverse interface. Ready to receive control commands.
   ```

   ROS can move the arm only after that line appears.

To check that the arm is live:

```bash
ros2 control list_controllers     # ur_arm_controller should be "active"
ros2 topic echo /joint_states     # should match the pendant's joint angles
```

## Moving the arm

Use the teach tool. It is a terminal or browser pendant that plans every
move through MoveIt, so every move is checked against the table first.

```bash
ros2 run bartender_teach teach        # terminal
ros2 run bartender_teach teach_gui    # browser, http://127.0.0.1:8080
```

### The first move

Before anything else, set the **speed slider on the pendant to 10-20%** and
keep a hand on the emergency stop. The slider really does slow ROS's moves
down.

The arm starts pointing straight up. From that pose it **cannot** move in a
straight line (`jog x` answers "only 0.00 of the path was reachable"), so
bend it with joint jogs first:

```
teach[a]> jog j2 20          # shoulder forward 20 degrees
teach[a]> jog j3 45          # elbow 45 degrees
teach[a]> save ready  bent, clear of the table
```

Now **check `arm_yaw`**, which is the check that matters most:

```
teach[a]> jog x 20
```

The tool must move **20 mm along the table's long side, away from the near
end** (the end the base is 0.35 from). If it moves any other way, the planner's
table is not where the real table is. Stop, work out the angle, and relaunch
with `arm_yaw:=<radians>` (for example `3.1416` if it went the opposite way).

Do not use the pendant's own Base jog for this check. The pendant's Base
frame is turned 180 degrees from the one ROS uses, so on the pendant +X moves
the tool *toward* the near end.

### Running the robot from our teach pendant

The teach pendant can also do the things you would otherwise walk over to
the UR pendant for. They work on the real robot only. In the simulation and
the dry run they say they are not available.

| Command | Browser button | Does |
|---|---|---|
| `robot` | Robot panel | Robot mode, safety mode, whether the program runs, speed |
| `robot on` | Power on | Power on and release the brakes (waits until done, up to ~30 s) |
| `robot play` | Play | Start the loaded program (`ros.urp`). ROS can move the arm only while it runs |
| `robot pause` / `robot stop` | Pause / Stop | Pause or stop that program |
| `robot unlock` | Unlock | Clear a protective stop, then `robot play` (or `robot resend`) again |
| `robot resend` | Resend script | Headless mode: start ROS's control script on the robot again |
| `robot off` | Power off | Power off (asks first in the browser) |
| `speed 20` | 10/25/50/100% | Set the UR speed slider, in percent |
| `freedrive on` / `off` | Freedrive | The arm goes limp and you move it by hand |

So a session can run without touching the UR pendant after the one-time
setup:

```
teach[a]> robot on
teach[a]> robot play
teach[a]> speed 15
teach[a]> robot            # program: running
```

**Freedrive** is the quickest way to teach: `freedrive on`, move the arm
with your hands, `save NAME`, then `freedrive off`. While it is on, the
pendant refuses jogs, `goto` and `run`. ROS's arm controller is switched off
and someone is holding the arm. Quitting the pendant turns freedrive off.

The emergency stop is still the red button on the UR pendant. Nothing here
replaces it.

### Everyday commands

| Command | Does |
|---|---|
| `state` | Current joint angles, tool position and gripper |
| `jog j1..j6 DEG` | Turn one joint (at most 45 degrees per command) |
| `jog x\|y\|z MM` | Straight line along the table's axes (x is along the table, z is up) |
| `jog tz MM` | Straight line along the gripper's pointing direction |
| `save NAME [note]` | Remember where the arm is now |
| `goto NAME` | Move to a saved point, on a path planned around obstacles |
| `movej NAME` | Move to a saved point, straight in joint space (like the UR's MoveJ) |
| `movel NAME` | Move the flange to a saved point in a straight line (like the UR's MoveL) |
| `list` | All saved points |
| `open` / `close [0-0.8]` | Gripper (does nothing on the real robot while it is simulated) |
| `help` | Everything else, including recording sequences |

By default, saved points go in `bartender_teach/config/taught_points.yaml`,
which belongs to the two-arm bar. **For this workcell, start the pendant
with `--file workcell`** so that its points go in
`bartender_teach/config/workcell_points.yaml` instead (see below). Points use
the same joint names as the simulation, so a point taught in sim can be
replayed on the real arm. Always replay it slowly the first time.

### Teaching the bottles (whiskey, vodka, jager)

```bash
ros2 run bartender_teach teach_gui --file workcell    # or: teach --file workcell
```

The file starts with only `home` (arm straight up). Each bottle gets three
points and one pipeline that picks it up. **The pipeline must be called
`grab_<bottle>`:** that name is how the API finds the bottles (see
"Picking a bottle through the API" below). Point names are up to you;
these are a suggestion:

| Point | Where the arm is |
|---|---|
| `<bottle>_pregrasp` | Gripper **open**, at grasp height, pointing at the bottle, about 100 mm short of it |
| `<bottle>_grasp` | Fingers around the bottle, just below its shoulder |
| `<bottle>_lift` | The same, 50 mm higher |

`<bottle>` is `whiskey`, `vodka` or `jager` (Jägermeister, which replaced
gin on 2026-09-27). Stand the three bottles where
they will always stand, and mark the spots on the table: a taught grasp is
only right while the bottle is back on its mark.

For each bottle (whiskey shown; repeat with `vodka` and `jager`):

```
teach[a]> speed 15
teach[a]> goto home
teach[a]> record grab_whiskey  whiskey from its spot
teach[a] rec:grab_whiskey> open
    ... bring the gripper in front of the bottle, 100 mm away, at grasp
    height (jog, or freedrive on / move it by hand / freedrive off) ...
teach[a] rec:grab_whiskey> save whiskey_pregrasp
teach[a] rec:grab_whiskey> jog tz 50          # straight in, in small steps
teach[a] rec:grab_whiskey> jog tz 50
teach[a] rec:grab_whiskey> safety off         # only if the planner refuses the last few mm
teach[a] rec:grab_whiskey> save whiskey_grasp
teach[a] rec:grab_whiskey> close
teach[a] rec:grab_whiskey> jog z 50
teach[a] rec:grab_whiskey> save whiskey_lift
teach[a] rec:grab_whiskey> goto home
teach[a] rec:grab_whiskey> stop
teach[a]> safety on
```

The recording gives `grab_whiskey`: open, go to the pre-grasp point, go to
the grasp point, close, lift, go home. To check it, run `run grab_whiskey dry`
(lists the steps, no motion), then `run grab_whiskey` at low speed. To put a
bottle back, visit the same points in reverse: `goto whiskey_lift`,
`goto whiskey_grasp`, `open`, `goto whiskey_pregrasp`.

### goto, movej or movel

All three end at a saved point, on the arm it was taught on. They differ in
the path they take to get there:

| | Path | Use it for |
|---|---|---|
| `goto` | Planned around obstacles (OMPL). Can take a different, roundabout path each time. | Long moves with things in the way |
| `movej` | Every joint turns at once and they all arrive together, like the UR's MoveJ. The same path every time. | Moves between approach points and home |
| `movel` | The flange travels in a straight line (UR MoveL, at up to 250 mm/s). | Going in to grab a bottle, lifting, pulling back out |

`movej` and `movel` do not go around obstacles. If the straight path
collides, or the arm can't follow it (a joint limit or singularity on the
way), they refuse **before** anything moves, and say why. Use `movej` or
`goto` from somewhere else in that case. The UR speed slider slows all three.

In a script, the step is written the same way: `{movel: grab_whiskey}`.
While recording, `movej NAME` and `movel NAME` add that kind of step. `save`
always adds a `goto`. To make a saved step a straight line, either:
- run `pipeline drop` then `pipeline step movel NAME` straight after the
  `save`, or
- change `goto:` to `movel:` in `workcell_points.yaml`.

The API's `/pick` and `/make` run scripts as they are, so they use
whichever kind each step is.

Things to know:

- **Jogs are not recorded**, only `save`, `goto`, `movej`, `movel`,
  `open`/`close` and `wait`.
  So jog as much as you like between saves.
- **`jog tz`** moves along the direction the gripper points. That is the
  approach direction, so the fingers slide straight onto the bottle.
- **The gripper is not driven on the real robot yet** (it is mocked until its
  wiring is decided). `open` and `close` are recorded but do not move the
  fingers, so make sure the fingers are physically open before approaching,
  and do not expect `whiskey_lift` to take the bottle with it yet.
- **Made a mistake?** Use `resave NAME` to overwrite a point, `pipeline drop`
  to remove the last step, or `pipeline rm grab_whiskey` and record again.
- The point file is rewritten on every save, and a rebuild does not touch
  it, because it lives in the source tree. Commit it once the three bottles
  are taught.

Full reference:
[`bartender_teach/README.md`](../ros2_ws/src/bartender_teach/README.md).

### Spout tools (for pouring)

Each bottle has a tool frame at its spout, so the arm can tilt the bottle
about the spout instead of the flange. Then the spout stays over the glass
while the bottle tips:

```
teach[a]> tool workcell_whiskey
teach[a]> jog ry 15          # tilts about the whiskey spout
teach[a]> tool tool0         # back to the flange
```

| Tool | Spout, from the grip point |
|---|---|
| `workcell_whiskey` | 70 mm up, 20 mm further along tz |
| `workcell_vodka` | 85 mm up, 15 mm further along tz |
| `workcell_jager` | 65 mm up, 20 mm further along tz (**not measured**: the gin bottle's numbers) |

The grip point is 145 mm in front of the flange along tz. The measurements
are `WORKCELL_SPOUT` in
[`tool_frames.py`](../ros2_ws/src/bartender_teach/bartender_teach/tool_frames.py). Only rotation jogs use the tool;
translation jogs and saved points are unaffected.

### Pouring and putting the bottle back

Whiskey and jager each have three scripts (2026-09-27), built from the taught
`aproach_<b>`, `grab_<b>`, `up_<b>`, `back_<b>` and `glass` points:

| Script | Does |
|---|---|
| `grab_<b>` | `gripper home` (gripper empty), open, `goto aproach_<b>`, `movel grab_<b>`, close until it grips, `movel up_<b>`, `movel back_<b>` |
| `pour_<b>` | `movej glass` (bottle upright), roll +90° about tool +z (the gripper's pointing axis) around the bottle TCP in six 15° `movel` steps, wait 3 s, tilt back to `glass`, still holding the bottle |
| `return_<b>` | `movej back_<b>`, `movel up_<b>`, `movel grab_<b>`, open, `movel aproach_<b>` |

So one pour is `run grab_whiskey`, `run pour_whiskey`, `run return_whiskey`,
which is also what the drinks menu runs for the spirit.

- **The bottle TCP is `workcell_bottle`**, the same for every bottle: 145 mm
  up (tool0 +X) and 162 mm ahead, horizontally (tool0 +Z), from tool0 in the
  side grasp. `WORKCELL_BOTTLE_TCP` in
  [`tool_frames.py`](../ros2_ws/src/bartender_teach/bartender_teach/tool_frames.py).
  The moves up to `glass` use the normal TCP (tool0).
- **The tilt points `glass_tilt_15` … `_90` are computed from `glass`**, so
  the spout stays in the same place during the whole tilt. Both bottles
  share them. **Re-teach `glass` and they must be recomputed.** After the
  file is edited outside the pendant, `reload` on the pendant re-reads it
  (otherwise its next save would undo the edit).
- **Which way it tips:** sideways, a roll about tool +z. Seen from the
  robot base looking at the glass, the neck tips to the RIGHT (base +x).
  Only wrist 3 turns much, 15° a step. (Tipping about tool y laid the neck
  back toward the arm, which was wrong; −z tips it to the left.)
- **How much it pours** is the `wait: 3.0` step in `pour_<b>` in
  `workcell_points.yaml`.
- **Getting to `glass` uses `movej`**: from `back_<b>` it keeps the bottle
  within about 3° of upright.
- The tilt points were computed offline (UR5e nominal kinematics) and have
  not run on the real arm yet. Run them slowly the first time, with an empty
  bottle.

### The gripper (MAB motor on the Pi)

The real gripper is a custom one, driven by one MAB motor (CAN ID 779)
connected to the Raspberry Pi at 10.42.0.200. Start its node on the Pi:

```bash
ssh bartender@10.42.0.200
cd ~/pi_code && ./run_arm.sh --ros-args -p "can_ids:=[779]"   # log: ~/mab_arm.log
```

The teach pendant (`teach`, `teach_gui`) and the API then use it by
themselves, but only after it has been **homed**. The fingers have no end
stop on the open side, and the drive's position shifts when it loses power,
so nothing is a fixed motor position: everything is measured from the
closed stop.

1. Empty the gripper.
2. `gripper home` on the pendant (or the **Home gripper** button). It
   closes the gripper until it stalls on its stop and stores that position
   in `~/.ros/bartender_mab_gripper.yaml`, which the pendant and the API
   share.
3. Home again whenever the Pi or the drive has been off. `open` refuses
   until it has been homed, and when the gripper reads below the stored
   stop (a sign its position has shifted).

Then `open` opens to 2.3 rad above the stop (the full stroke is about 2.4),
and `close` or any `grip` step aims 0.2 below the stop, so it closes until
the fingers stall, on the bottle or on the stop, pushing with up to the
drive's `maxTorque` of 3 Nm (enough to hold a bottle, not enough to crack
the glass). There are no in-between positions. After the stall `candletool`
switches the drive off, so the fingers stay put but are no longer pushed.
Each command waits 2 s for the fingers. To force a choice:
`--gripper mab` or `--gripper robotiq` (the simulated one).

Only give `can_ids:=[779]`: with the default two motors the node ignores
one-value commands, and polling the missing motor crashes `candletool`.

**Drive gains (2026-09-27).** With the drive's stock gains, closing
stopped partway (for example 2.79 → 2.62): near the target the velocity
loop pushed with only about 0.45 Nm, far below the 7 Nm `maxTorque`. The
gains were raised on the drive (`candletool md -i 779 register write ...`,
then `candletool md -i 779 save`):

| Register | Stock | Now |
|---|---|---|
| `motorVelPidKp` | 0.1 | 0.2 |
| `motorVelPidWindup` | 0.25 | 1.5 |
| `motorPosPidKp` | 10.5 | 15 |
| `maxTorque` | 7 | 3 (grip force: holds a bottle, does not crack it) |

If it buzzes or oscillates, lower `motorVelPidKp` first. `config download`
crashes in candletool 1.5.0, so this table is the record of the old values.
The drive still carries the `MA-p-45-10_KV75` motor profile; the motor is
an MA-p-40-10. The open travel and the homing sweep are in
[`mab_gripper.py`](../ros2_ws/src/bartender_teach/bartender_teach/mab_gripper.py).

### Picking a bottle through the API

Full reference, for whoever writes the calling program: [WORKCELL_API.md](WORKCELL_API.md).

Once a bottle has its `grab_<bottle>` pipeline, other programs can ask for
it over HTTP. Start the API server with this cell's point file, next to a
running `workcell_twin` (or `workcell_real`):

```bash
ros2 run bartender_api server --points workcell
```

```bash
curl http://127.0.0.1:8090/bottles
# {"bottles": [{"bottle": "whiskey", "pipeline": "grab_whiskey", "steps": 6}], "points_file": "..."}

curl -X POST http://127.0.0.1:8090/pick \
  -H 'Content-Type: application/json' -d '{"bottle": "whiskey"}'
# {"ok": true, "bottle": "whiskey", "message": "  running grab_whiskey: ... finished grab_whiskey"}
```

- `/pick` runs `run grab_<bottle>` exactly as the pendant would, and answers
  when the pipeline has finished or stopped. The HTTP status is 200 if every
  step worked and 409 if not; `message` says which step stopped and why.
- A bottle nobody has taught gets a 409 that lists the bottles that do
  exist. **To add a bottle, teach `grab_vodka` / `grab_jager` on the
  pendant.** The server re-reads the point file on every request, so there
  is nothing to restart and no code to change.
- One command at a time: a `/pick` while another move is running is
  refused (`busy`), not queued. The browser pendant is a separate process
  and is *not* locked out, so don't drive the arm from both at once.
- The speed is whatever the UR speed slider is set to (`speed` on the
  pendant).

### Making a drink through the API

A drink is one API call that runs a list of scripts, in order. The menu is
[`bartender_teach/config/workcell_menu.yaml`](../ros2_ws/src/bartender_teach/config/workcell_menu.yaml):
whiskey, jager and vodka, each with cola, sprite or fanta. Each drink runs
three scripts per ingredient, which are yours to teach:

| Script | Does |
|---|---|
| `grab_<x>` | Pick the bottle up from its spot (the same one `/pick` runs) |
| `pour_<x>` | Pour it into the glass |
| `return_<x>` | Put it back on its spot |

`<x>` is `whiskey`, `jager`, `vodka`, `cola`, `sprite` or `fanta`. A
drink is "not ready" until all of its scripts and their points are
taught. The server reads the menu with `--points workcell`.

```bash
curl http://127.0.0.1:8090/drinks
# each drink: "ready": true/false, and "missing_scripts" for what to teach next

curl -X POST http://127.0.0.1:8090/make \
  -H 'Content-Type: application/json' -d '{"drink": "whiskey_cola"}'
```

- **Nothing moves unless every script exists** and every point those
  scripts use exists. A drink that isn't ready gets a 409 listing what is
  missing.
- **It stops at the first script that doesn't finish.** The answer then has
  `"ok": false` and `"stopped_at": "script 4 of 6: grab_cola"`. Nothing puts
  back a bottle that is still in the gripper.
- **To change a drink or add one**, edit the menu file. The order and names
  of scripts are entirely up to you; the next request picks up the change.

## Stopping, and recovering

- **Emergency stop or protective stop**: the pendant stops the arm, and
  ROS's arm controller is stopped automatically, so an interrupted move does
  **not** resume by itself.
- **To carry on**: clear the stop on the pendant (or run
  `ros2 service call /dashboard_client/unlock_protective_stop std_srvs/srv/Trigger`),
  then press **Play** again. The controllers come back on their own when the
  program is running.
- **To finish**: press Ctrl+C in the launch terminal, then stop the program on
  the pendant.

The pendant can also be driven from the PC:

```bash
ros2 service call /dashboard_client/power_on     std_srvs/srv/Trigger
ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
ros2 service call /dashboard_client/play         std_srvs/srv/Trigger
ros2 service call /dashboard_client/stop         std_srvs/srv/Trigger
```

## When something is wrong

| Symptom | Likely cause |
|---|---|
| Launch hangs at connecting, or "Connection refused" | Cable, or step 2. `ping 10.42.0.100` first. |
| Every move fails with `error_code -4`, or the pendant says "the control program is not running" | The program on the robot is not running, so the driver has switched `ur_arm_controller` off. In Remote Control mode: make sure you did not launch with `headless_mode:=false`, then `robot resend` if needed. In Local mode: load and play the External Control program. |
| The arm starts, stops, starts again ("bouncing"), then `error_code -6`; move_group logs "Controller is taking too long to execute trajectory" | MoveIt is cancelling moves that the speed slider has slowed down. `workcell_real` and `workcell_twin` turn that check off. If you still see it, you are running a build or launch from before 2026-09-25: rebuild and relaunch. |
| `robot ...`: "not available" | This is the sim or the dry run, or the driver is not connected yet. |
| `freedrive on` fails | The program is not running (`robot play`), or `ur_arm_controller` is not active. |
| Gazebo twin does not move | `ros2 topic hz /joint_states` for the real side. `twin_mirror` logs a warning when it stops hearing the robot. |
| Program plays on the pendant but "Ready to receive control commands" never appears | URCap Host IP is not this PC's address (step 3), or the firewall (step 4). |
| `jog x`: "only 0.00 of the path was reachable" | The arm is straight up. Joint-jog it into a bend first. |
| Move refused, log mentions `workcell_table` | The move would hit the table. If the table is clearly not in the way, `arm_yaw` or `table_height` is wrong. |
| MoveIt says success but the arm does not move | The speed slider is at 0%, or the program is paused on the pendant. |
| Gripper "closes" but nothing happens | Is `run_arm.sh` running on the Pi? Without it, the pendant falls back to the simulated Robotiq. See "The gripper". |
| `package 'ur_robot_driver' not found` | Step 1. |

## All launch arguments

`workcell_real.launch.py`:

| Argument | Default | |
|---|---|---|
| `robot_ip` | `10.42.0.100` | The robot's IP |
| `use_fake_hardware` | `false` | `true` = dry run, no robot |
| `arm_yaw` / `table_height` | `-1.5708` / `0.75` | See step 5 |
| `arm_x` / `arm_y` | `0.35` / `0.35` | Base position from the table corner |
| `gripper_fake_hardware` | `true` | `false` to drive the real gripper |
| `use_tool_communication` | `false` | `true` if the gripper is wired to the UR's tool connector |
| `tool_voltage` | `0` | Tool connector voltage (24 for a Robotiq on the tool connector) |
| `headless_mode` | `true` | `true` = no pendant program needed; the robot must be in Remote Control mode. `false` = Local mode with the External Control program |
| `reverse_ip` | `0.0.0.0` | This PC's IP as the robot sees it; the default works it out |
| `launch_moveit` | `true` | |
| `launch_rviz` | `false` | RViz currently throws a display error on startup; see the main README |

`workcell_sim.launch.py` takes `arm_x`, `arm_y`, `arm_yaw`, `table_height`,
`headless` and `launch_rviz`.

`workcell_twin.launch.py` takes everything `workcell_real.launch.py` does, plus
`headless` (twin Gazebo without its window).

## Where things are

| File | What |
|---|---|
| `bartender_description/urdf/workcell.urdf.xacro` | Arm, gripper and table |
| `bartender_description/config/workcell_controllers.yaml` | Controllers in sim |
| `bartender_description/config/workcell_ur_controllers.yaml` | Controllers on the real robot |
| `bartender_moveit_config/srdf/workcell.srdf` | MoveIt groups and allowed contacts |
| `bartender_bringup/launch/workcell_sim.launch.py` | Sim bringup |
| `bartender_bringup/launch/workcell_real.launch.py` | Real / dry-run bringup |
| `bartender_bringup/launch/workcell_twin.launch.py` | Real robot + Gazebo twin |
| `bartender_bringup/bartender_bringup/twin_mirror.py` | Copies the real arm onto the twin |
| `bartender_description/config/workcell_twin_controllers.yaml` | The twin's controllers (follow only) |
| `bartender_teach/bartender_teach/robot_control.py` | The teach pendant's power / program / speed / freedrive |
| `bartender_gazebo/worlds/workcell_world.sdf` | Gazebo world (floor and light) |
