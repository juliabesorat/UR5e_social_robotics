# UR5e pose sequences with MoveIt 2

This optional example uses the existing `handshake.yaml` to compare two ways
to move the UR5e. The new node runs independently from the classroom sequence
server. The existing server and its normal launch are unchanged.

## 1. The existing sequence node

`ur5e_pose_sequence.py` reads `common` and `steps` from the YAML file. For each
step it:

1. Converts position from millimetres to metres and orientation from degrees
   to a quaternion.
2. Transforms the target from `table` to `base_link` using TF.
3. Calls MoveIt's `/compute_ik` service to find joint angles. It uses the
   current joint state or the YAML seed as the IK starting guess.
4. Sends the angles as one trajectory point to
   `/scaled_joint_trajectory_controller/follow_joint_trajectory`.
5. Waits for the controller result and continues with the next step.

The controller interpolates the joint motion over the step's `duration`.
MoveIt provides inverse kinematics, but its trajectory planner is not asked
to find the path between poses.

### Limitations

- MoveIt does not plan or check the interpolated path against the planning
  scene. The IK request does not explicitly enable collision checking.
- The node does not use `max_velocity` or `max_acceleration` from the YAML.
  `duration` specifies a time, not MoveIt scaling factors.
- With `execute: false`, the node calculates IK but does not create a
  trajectory to preview in RViz.
- A successful IK solution does not guarantee a collision-free path to it.

## 2. The new MoveIt sequence node

`ur5e_pose_sequence_moveit.py` is a separate executable in the same package.
It reads the same pose fields and uses the same coordinate transformation.
For each step it sends a pose goal and a start state to MoveIt's `/move_action`.
The goal has position and orientation tolerances. MoveIt returns a planned
joint trajectory, or an error if planning fails.

The node publishes accumulated trajectories on `/display_planned_path` for
RViz. In preview mode, each step starts from the planned end state of the
previous one. In execution mode, it sends each plan to MoveIt's
`/execute_trajectory` action, waits for the result, and uses the current joint
state for the next plan. A failed action or an empty plan stops the sequence.

| Setting | Existing node | MoveIt sequence node |
| --- | --- | --- |
| Target | IK solution sent to the controller | Pose sent to the MoveIt planner |
| Path | Controller interpolates one final point | MoveIt plans a joint trajectory |
| Collision checking | No MoveIt path planning or path check | Uses the robot model and planning scene |
| Timing | `duration` from the YAML | MoveIt trajectory timing and scaling factors |
| Preview | IK only when `execute: false` | Plans without executing when `execute:=false` |

The new node uses `group_name`, `ik_link`, `target_frame`, `planning_frame`,
`max_velocity`, `max_acceleration` and `sleep_after`. Scaling factors must be
greater than 0 and at most 1. The launch argument `execute` controls execution;
the YAML `execute` field is ignored. The node does not use `duration`,
`seed_joints`, `seed_from_joint_states`, `ik_timeout_sec` or `print_joints`.
It uses free-space planning for all steps; it does not guarantee a straight
Cartesian path. Planning to a pose does not guarantee that every IK solution
will be explored.

## 3. Build the workspace

These commands use the ROS 2 Humble environment in the
[setup guide](UR5e_ROS2_Setup.md). Run them on the computer or inside the
container where the workspace and UR driver are installed. They assume the
repository is at `~/UR5e_social_robotics`; change the path if necessary.

Open a build terminal:

```bash
cd ~/UR5e_social_robotics
source /opt/ros/humble/setup.bash
source .venv-humble/bin/activate
.venv-humble/bin/python /usr/bin/colcon build --packages-up-to ur5e_robot_controller
source install/setup.bash
ros2 pkg executables ur5e_robot_controller
```

Check that the output includes `ur5e_pose_sequence_moveit_exe` and
`ur5e_test_scene_exe`. If the
workspace or dependencies are not yet prepared, follow the setup guide first.
Use the same ROS domain and network settings in every terminal; see
[Session 2](../../4a_ROS2_Session.md) for the classroom settings.

## 4. Start the fake robot and MoveIt

Keep the following terminals open. In each new terminal, prepare the same
environment before running its command:

```bash
cd ~/UR5e_social_robotics
source /opt/ros/humble/setup.bash
source .venv-humble/bin/activate
source install/setup.bash
```

**Terminal 1 — fake UR5e driver:**

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=192.168.0.20 \
  use_fake_hardware:=true launch_rviz:=false
```

**Terminal 2 — MoveIt and RViz:**

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e launch_rviz:=true
```

Wait until the driver, MoveIt and RViz are ready.

The sequence launch in section 7 publishes the `base_link` to `table` transform,
using the same 180-degree rotation around the vertical axis as the existing
sequence launch. Stop any other publisher of this transform before starting
the new launch. The transform defines a frame, not a physical table in the
MoveIt planning scene.

## 5. Check the interfaces

In **terminal 3**, prepare the environment as above and run:

```bash
ros2 topic echo /joint_states --once
ros2 action list | grep -E '^/(move_action|execute_trajectory)$'
```

The first command should show joint names and positions; the second should
show both actions. If an interface is missing, check the earlier terminals and
the ROS domain. The `table` transform will appear when the sequence launch runs.

## 6. Add the test table

The separate `ur5e_test_scene` node adds one collision object to MoveIt's
planning scene. It does not change the robot's URDF, the sequence server, or
the existing classroom launches. The box is 1 m by 1 m and 0.05 m thick.
Its centre is `(0, 0, -0.025)` in `base_link`, so its top is at `z = 0`.
This assumes that `base_link` is at the table surface: check this assumption
against the model and the real installation before any real robot test.

The planning scene belongs to the running MoveIt instance. All clients of
that instance can see this table. Do not run this test while a class is using
the same MoveIt instance. It is not started by any classroom launch.

In **terminal 3**, with the environment prepared as in section 4, run:

```bash
ros2 run ur5e_robot_controller ur5e_test_scene_exe
```

Wait for `Added ur5e_test_scene_table`. Keep this terminal open while testing.
In RViz, check that the MotionPlanning display shows scene geometry and that
the table is centred under the robot. Check that the robot's starting pose and
the targets are not in collision with the box. Contact or overlap with the
robot base can cause planning to fail; do not change collision permissions
without inspecting the geometry.

When you finish testing, press Ctrl+C in terminal 3 **while MoveIt is still
running**. The node requests removal of only `ur5e_test_scene_table` and
reports whether it succeeded. Check that the table disappears from RViz.
If the node is killed unexpectedly, or removal fails, restart MoveIt or, with
MoveIt running and no test scene node active, remove the stale test table:

```bash
ros2 run ur5e_robot_controller ur5e_test_scene_exe --ros-args -p remove_only:=true
```

This command removes only the object with that ID. The node refuses to replace
an existing object with the same ID when it starts normally. Do not use the
cleanup command while another test scene node is active.

## 7. Plan without moving

Keep terminal 3 running. Open **terminal 4**, prepare its environment as in
section 4, and run:

```bash
ros2 launch ur5e_robot_controller ur5e_pose_sequence_moveit.launch.py \
  sequence_file:=handshake.yaml execute:=false
```

The terminal should report `Planning step` and `Completed step` for each of
the five steps, followed by `Sequence finished.` The fake robot's joint state
does not change. In RViz, enable the MotionPlanning display and its planned
path visualization before launching the command; watch the planned sequence as
the node publishes it. Run the command again if RViz was not ready in time.
The launch reads the installed YAML from `social_robot_behaviors`, not an
arbitrary path. Rebuild or install YAML changes before testing them.

To inspect the transform while the launch is running, prepare another terminal
and run `ros2 run tf2_ros tf2_echo base_link table`. Stop this check with
Ctrl+C. The transform publisher exits when you stop the sequence launch.

## 8. Execute with fake hardware

After reviewing the preview, use the same terminal:

```bash
ros2 launch ur5e_robot_controller ur5e_pose_sequence_moveit.launch.py \
  sequence_file:=handshake.yaml execute:=true
```

Watch the fake robot move in RViz and check that all five steps complete.
The node replans each step, so the executed trajectory may differ from the
preview. `sleep_after` is applied after each successful execution. If planning
or execution fails, the node reports an error and stops; inspect the terminal
and MoveIt logs before retrying. Stop the launched processes with Ctrl+C when
you finish.

## 9. What the planner can and cannot see

MoveIt checks the robot model and objects present in its planning scene. The
`table` TF frame does not add a table collision object. Objects that are not
modelled cannot be avoided by the planner. The optional test scene adds only
the table. It does not detect moving people or guarantee avoidance of singularities.
A preview and low speed do not replace the safety measures required for a
real robot; start with fake hardware.
