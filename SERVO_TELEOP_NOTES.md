# PandaMJK — Servo Teleop: Architecture & Debugging Notes

Working notes for the MoveIt Servo–based Cartesian teleoperation stack in
`panda_mjk`. Captures the pipeline, config decisions, hard-won fixes, and the
currently-open "only moves in one direction" bug.

Last updated: 2026-08-13.

---

## 1. What this system is

A unified ROS 2 Humble package (`panda_mjk`) for controlling a **Franka Emika
Panda** arm, switchable between:

- **MuJoCo simulation** (`mujoco_ros2_control`) — default, `use_sim:=true`.
- **Real hardware** (`franka_hardware` / libfranka FCI) — `use_sim:=false`.

Cartesian teleop is done through **MoveIt Servo**, which converts Cartesian
twist commands into streamed joint-position commands for a `ros2_control`
position controller.

### Robot naming (important, easy to trip over)
- The real Franka Emika Panda is identified as **`fer`** in Humble
  `franka_description` (the old `panda` arm_id is deprecated; `fr3` is a
  *different*, newer robot with different kinematics — do **not** use it).
- `no_prefix:=true` is passed to the `franka_description` macros so both sim and
  real expect bare joint names **`joint1`..`joint7`** (matching DeepMind's
  `mujoco_menagerie` model).
- Planning group: **`arm`** (base `link0`, tip `link8`) — **not** `panda_arm`.
- Known SRDF wrinkle: `franka_robot.xacro` doesn't forward `no_prefix` to the
  hand attachment, so the hand/finger links come out `fer_`-prefixed in the URDF
  while `fer.srdf.xacro`'s hand group comes out bare — a mismatch that makes
  MoveIt fail to load those links. We pass **`hand:=false`** to the SRDF for
  Servo since Cartesian *arm* servoing doesn't need the hand group (the real
  gripper is driven separately via `franka_gripper`).

---

## 2. The control pipeline

```
                 (drag + release marker)            (Cartesian twist)
  RViz Interactive Marker ──► servo_interactive_marker ──► /servo_node/delta_twist_cmds
                                (P-controller on TF error,      │
                                 emits TwistStamped in link0)   ▼
                                                         MoveIt Servo (servo_node)
                                                         - IK / Jacobian
                                                         - command_in_type: speed_units
                                                         - outputs joint POSITIONS
                                                                │
                                              std_msgs/Float64MultiArray
                                                                ▼
                                       /forward_position_controller/commands
                                                                │
                                          forward_command_controller/ForwardCommandController
                                                     (interface_name: position)
                                                                ▼
                                    ros2_control  ──►  MuJoCo position actuators  (or real FCI)
```

Joystick path is analogous but the source is `cartesian_teleop` (joy → twist).

---

## 3. Key files

| File | Role |
|------|------|
| `src/panda_mjk/urdf/panda.urdf.xacro` | URDF; selects `mujoco_ros2_control` vs `franka_hardware` system on `use_sim` |
| `src/panda_mjk/config/controllers.yaml` | `controller_manager` @ 1 kHz; `joint_state_broadcaster`, `forward_position_controller`, `panda_gripper_controller` |
| `src/panda_mjk/config/servo_parameters.yaml` | MoveIt Servo params |
| `src/panda_mjk/config/kinematics.yaml` | KDL solver for group `arm` |
| `src/panda_mjk/config/mujoco_pids.yaml` | Intentionally **no** PIDs (see §5) |
| `src/panda_mjk/config/teleop.rviz` | RViz; marker Update Topic `/servo_interactive_marker/update` |
| `src/panda_mjk/src/servo_interactive_marker.cpp` | **C++** interactive-marker → Servo bridge (current) |
| `src/panda_mjk/src/cartesian_teleop.cpp` | Joystick → Servo twist bridge + gripper action |
| `src/panda_mjk/launch/panda_control.launch.py` | Brings up control stack (URDF, controller_manager, spawners, RSP) |
| `src/panda_mjk/launch/servo.launch.py` | Starts `servo_node` **and calls `start_servo`** |
| `src/panda_mjk/launch/teleop.launch.py` | Joystick teleop + servo |
| `src/panda_mjk/launch/visual_teleop.launch.py` | RViz + `servo_interactive_marker` + servo |

### Launch bring-up order (sim)
```bash
ros2 launch panda_mjk panda_control.launch.py          # control stack + MuJoCo
ros2 launch panda_mjk visual_teleop.launch.py          # RViz marker teleop
#   or
ros2 launch panda_mjk teleop.launch.py                 # joystick teleop
```

---

## 4. Config specifics that matter

### `controllers.yaml`
- `controller_manager.update_rate: 1000`  (1 kHz for FCI).
- `forward_position_controller`: `forward_command_controller/ForwardCommandController`,
  `interface_name: position`, joints `joint1..joint7`.
  (Functionally equivalent to the plan's `position_controllers/JointGroupPositionController`;
  the ForwardCommandController variant is what's proven working here.)
- `panda_gripper_controller`: `position_controllers/GripperActionController`, sim only.

### `servo_parameters.yaml`
- `move_group_name: arm`, `planning_frame: link0`, `ee_frame_name: link8`,
  `robot_link_command_frame: link0`.
- `command_in_type: speed_units`  ← twist values are **real m/s and rad/s**
  (deliberate; commit 972d479). In this mode `scale.*` is **not** applied.
- `command_out_type: std_msgs/Float64MultiArray`,
  `command_out_topic: /forward_position_controller/commands`.
- `publish_joint_positions: true`, `publish_joint_velocities: false`.
- `publish_period: 0.01` (100 Hz Servo loop).
- `incoming_command_timeout: 0.1` ← **Servo halts if no fresh command within
  100 ms.** Command staleness is judged from `header.stamp`. Critical for the
  timestamp gotcha in §6.
- `check_collisions: true`, singularity thresholds
  `lower: 17.0`, `hard_stop: 30.0`.

---

## 5. Hard-won lessons (from git history + code comments)

These are already baked into the code; don't regress them.

1. **`start_servo` must be called.** Servo starts *paused* and emits nothing on
   `command_out_topic` until `/servo_node/start_servo` (Trigger) is called.
   `servo.launch.py` does this via `ExecuteProcess` (the service call blocks
   until available, so no explicit delay is needed). (commit 670a4b7)
2. **`use_sim_time` must match everywhere.** MuJoCo's `ros2_control_node` runs
   with `use_sim_time:=true`, so `/joint_states` carry sim-time stamps. Servo,
   the teleop node, and RViz must all use sim time or command stamps look stale
   → "runaway velocity timeout" halts. (commit dc61c46)
3. **Stream velocity, not absolute pose.** `ros-humble-moveit-servo` does not
   reliably drive the arm from its `pose_target_cmds` topic. A released target
   is reached by continuously re-computing a proportional velocity toward it.
   (commits 3938b59 → ee6ca96 → 9013b21 — this thrashed back and forth.)
4. **Bound the seek.** When streaming toward a released target, give up after
   `seek_timeout` (5 s). Otherwise an unreachable target (outside workspace)
   means the error never shrinks and we stream max-speed commands forever.
   (commit ebc4f80)
5. **Anti-flicker marker sync.** When idle, snap the marker back to the live EE
   only when the pose *meaningfully* changed. Calling `setPose`/`applyChanges`
   every 50 Hz tick rebuilds the marker's clickable object in RViz constantly →
   flicker + unclickable. (commit 4ccec7c)
6. **No MuJoCo software PIDs.** `mujoco_pids.yaml` is intentionally blank of
   `pids`. The menagerie `panda.xml` uses `<position>` actuators, so
   `mujoco_ros2_control` should pass position commands straight through; a
   software PID loop would compute an effort and wrongly set it as the position
   setpoint → physics instability. (commit 4e5eed3)
7. RViz-specific: RobotModel not rendering (0f16341), TF view disabled by
   default + correct interactive-markers topic (c94bd31).

---

## 6. The `servo_interactive_marker` node (current C++ bridge)

Reimplemented from the earlier Python `interactive_teleop.py` (now deleted;
git preserves it). Same behaviour, C++.

**Behaviour:**
- Spawns a 6-DOF `InteractiveMarkerServer` in namespace `servo_interactive_marker`
  (→ topic `/servo_interactive_marker/update`, matched in `teleop.rviz`).
- **Drag** repositions the marker only (`is_dragging_` true on `POSE_UPDATE`).
- **Release** (`MOUSE_UP`) sets `pending_target_`, records `seek_start_time_`.
- While a target is pending, a 50 Hz loop computes Cartesian error from live TF
  (`link0`→`link8`), runs a P-controller (`kp_linear=5`, `kp_angular=2`,
  clamped to `max_linear_vel=0.2`, `max_angular_vel=0.5`), and publishes
  `TwistStamped` (frame `link0`, stamp `this->now()`) to
  `/servo_node/delta_twist_cmds` until within tolerance or `seek_timeout`.
- When idle, snaps marker back to the live EE (with the anti-flicker guard).

**Params:** `base_frame` (link0), `ee_frame` (link8), `kp_linear`, `kp_angular`,
`max_linear_vel`, `max_angular_vel`, `position_tolerance` (0.005),
`orientation_tolerance` (0.05), `seek_timeout` (5.0).

**Orientation error:** `q_err = q_target * q_current.inverse()`, converted to a
rotation vector (`axis * angle`), angle wrapped to the shorter way around.

**Build:** compiles cleanly in the project Docker image against ROS 2 Humble
(verified `colcon build --packages-select panda_mjk` → `servo_interactive_marker`
binary produced). Deps added: `visualization_msgs`, `interactive_markers`,
`tf2`, `tf2_ros`, `tf2_geometry_msgs`.

---

## 7. OPEN BUG — "teleop only moves in one direction"

**Symptom (reported on real testing):** With the marker, the EE moves but only
in one direction; it won't track the marker the other way.

### The timestamp red herring (resolved)
A manual `ros2 topic pub` twist test appeared to "not move at all" — but that
test is **invalid under sim time**: `ros2 topic pub` sends `header.stamp = 0`,
and with `use_sim_time:=true` Servo sees it as infinitely old and drops it
(`incoming_command_timeout: 0.1`). The real node stamps with `this->now()`, so
its commands are fresh. **Conclusion: don't diagnose from zero-stamp manual
pubs on a sim-time system.** To manually inject twist, publish with a live stamp
(e.g. a small script) or drive the marker.

### Current working theory
Because the bug **survived a full reimplementation** of the teleop node
(Python → C++, identical logic), the fault is almost certainly **downstream of
the twist**, i.e. in Servo, the controller, or the sim — not in the marker /
P-controller logic.

Leading hypotheses, most likely first:
1. **Servo self-halting** on singularity / joint limit / collision — arm can
   move *away* from a bound but not back through it. Check `/servo_node/status`
   (nonzero = halting).
2. **Stale / frozen joint feedback into Servo** — if `/joint_states` don't
   update (a `use_sim_time`/clock issue), Servo integrates from a fixed "current"
   position and marches one way until a limit.
3. **Joint-order mismatch** between Servo's output array and the controller's
   `joints:` list (usually reads as chaotic, not clean one-direction — lower
   likelihood).
4. Controller→MuJoCo actuator mapping / not claiming command interfaces.

### Diagnostics to run (all sim-time-safe)
1. **Servo output on the real path** — while dragging+releasing the marker +X
   then −X:
   ```bash
   ros2 topic echo /forward_position_controller/commands
   ```
   Do `data:` values go up then back down (tracks both ways), or only drift one
   way?
2. **Servo status:**
   ```bash
   ros2 topic echo /servo_node/status
   ```
3. **Isolate controller from Servo** (no timestamp confound) — read joints, then
   nudge one straight to the controller:
   ```bash
   ros2 topic echo /joint_states --once      # 7 arm joints, joint1..joint7 order
   ros2 topic pub -1 /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [q1, q2, q3, q4+0.3, q5, q6, q7]}"
   ```
   - Moves → controller + MuJoCo fine; fault is inside Servo.
   - No move → fault is controller→MuJoCo.
4. **Sanity:**
   ```bash
   ros2 control list_controllers        # forward_position_controller active?
   ros2 topic hz /joint_states          # updating? (feeds Servo)
   ros2 service call /servo_node/start_servo std_srvs/srv/Trigger   # re-arm
   ```

**Next step:** get `data:` for +X vs −X from test 1 and any nonzero
`/servo_node/status` — that pins which side of Servo the bug is on.

---

## 8. Environment / build

- Docker: `osrf/ros:humble-desktop` base; **host networking**, `privileged`,
  `SYS_NICE`, `rtprio: 99` (1 kHz FCI + DDS discovery). `MUJOCO_GL=glfw`, X11
  passthrough for the GUI.
- Workspace mounts at `/ros2_ws/src` (compose bind mount `./src`).
- Build one package:
  ```bash
  colcon build --packages-select panda_mjk
  ```
- If a stale install artifact bites (e.g. leftover `interactive_teleop.py`
  symlink from a pre-rebuild image), `rm -rf build install log` then rebuild.

---

## 9. Git state

Latest relevant commits (newest first):
```
2ce09da feat: reimplement interactive-marker teleop as a C++ servo bridge
ebc4f80 fix: bound how long teleop keeps seeking a released target
9013b21 fix: revert to twist-velocity streaming toward released target
ee6ca96 feat: move to target pose on marker release instead of live velocity servoing
972d479 fix: change servo to speed_units and correct rviz marker topic
dc61c46 fix: add use_sim_time to teleop node to fix runaway velocity timeout bug
3938b59 fix: publish direct PoseStamped position commands instead of Twist velocities
```
Remote: `git@github.com:Altimerra/pandamjk.git` (`origin/main`).
