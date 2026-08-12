# UR5e MoveIt 2 Motion Planning Lab

A ROS 2 Humble workspace for learning MoveIt 2 in depth on a UR5e, using
fake (mock) hardware -- no physical robot or Gazebo required. Covers joint
and pose goals, Cartesian-style paths via Pilz LIN, obstacle avoidance,
OMPL planner comparison, the Pilz Industrial Motion Planner (PTP/LIN/CIRC),
orientation constraints, and attach/detach for pick-and-place.

## 1. Workspace layout

```
ur5e_moveit_ws/
└── src/
    └── ur5e_motion_planning_lab/    # this package -- the only thing you build
```

`ur_robot_driver`, `ur_moveit_config`, and `ur_description` are **prebuilt
Debian binaries** on the standard ROS package server -- installed via
`apt`, not cloned/built from source. There's no reason to build them
yourself unless you need to patch driver internals. `ur_moveit_config`
already ships a complete MoveIt setup for every UR model (SRDF,
kinematics.yaml, OMPL config, controller config); this package
(`ur5e_motion_planning_lab`) is where your actual learning/experimentation
code lives, sitting on top of that.

## 2. Setup

```bash
# Install the UR metapackage -- pulls in ur_robot_driver, ur_moveit_config,
# ur_description, ur_controllers, ur_dashboard_msgs, etc. as binaries
sudo apt update
sudo apt install ros-humble-ur

export COLCON_WS=~/ur5e_moveit_ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS

# This package -- copy/clone it into src/ur5e_motion_planning_lab
# (however you're transferring these files from this conversation)

rosdep update
rosdep install --ignore-src --from-paths src -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If `colcon build` fails on `ur5e_motion_planning_lab` specifically with a
missing `moveit_py` or `ament_index_python` error, confirm you have
`ros-humble-moveit-py` installed (`sudo apt install ros-humble-moveit-py`
if not) -- it's a separate binary package from `ros-humble-moveit` and from
`ros-humble-ur`.

**When you'd actually need to clone the driver repo instead**: only if
you're modifying driver/MoveIt-config internals yourself, need a bugfix
that's merged on `humble` but not yet released to the binary package, or
want to point `ur_type` at a robot variant not yet in the released
`ur_description`. None of that applies to this lab -- `apt install
ros-humble-ur` is sufficient and is what the rest of this README assumes.

## 3. Run it


**Terminal 1** -- bring up fake hardware + MoveIt + RViz:

```bash
source install/setup.bash
ros2 launch ur5e_motion_planning_lab ur5e_fake_moveit.launch.py
```

Wait for RViz to open and the robot model to appear before proceeding --
this confirms `/joint_states` is flowing and the planning scene monitor has
initialized.

**Terminal 2** -- run any experiment:

```bash
source install/setup.bash
ros2 run ur5e_motion_planning_lab ex1_joint_goal
```

## 4. Suggested learning order

| # | Script | Concept |
|---|--------|---------|
| 1 | `ex1_joint_goal` | Joint-space goals, explicit `RobotState` targets |
| 2 | `ex2_pose_goal` | Task-space pose goals, implicit IK |
| 3 | `ex3_cartesian_path` | Chained Pilz LIN segments for straight-line paths |
| 4 | `ex4_collision_objects` | Adding/removing obstacles, watching OMPL route around them |
| 5 | `ex5_planner_comparison` | RRTConnect vs RRT* vs PRM* vs EST vs KPIECE on the same problem |
| 6 | `ex6_pilz_linear` | Pilz PTP vs LIN, mapped onto ABB RAPID MoveJ/MoveL if that helps |
| 7 | `ex7_orientation_constraints` | Constrained planning (keep the gripper level) |
| 8 | `ex8_attach_detach_object` | Attach/detach for pick-and-place, touch_links |

Each script is self-contained and independently runnable/rereadable --
they're not meant to be imported as a library, they're meant to be read
top-to-bottom while RViz is open next to them.

## 5. Where to go from here

Once these feel natural:

- **CHOMP / STOMP**: gradient-based / stochastic trajectory optimizers,
  not covered here -- add `"chomp"` to the `pipelines=[...]` list in
  `mgi_base.py`'s `build_moveit_config()` and a matching named block in
  `config/moveit_py_params.yaml` to compare against OMPL.
- **MoveIt Task Constructor (MTC)**: the proper tool once you're chaining
  multiple stages (approach, grasp, retreat, place) as a single planned
  task rather than independent scripted segments like `ex8` does here.
  Given your CASE 2026 work on execution-stage failure attribution, MTC's
  stage-level success/failure reporting is worth a serious look for your
  dissertation testbed.
- **Perception-driven collision scenes**: replace the hardcoded
  `make_box_collision_object()` calls in `ex4`/`ex8` with objects populated
  from your SAM3 segmentation output -- this is the natural bridge back
  into your existing palletization pipeline.
- **Real hardware / URSim**: everything here transfers directly -- swap
  `use_fake_hardware:=true` for a real `robot_ip` (or a running URSim
  Docker container), same MoveIt config, same experiment scripts unchanged.

## 6. Known caveats (read before debugging blind)

I verified the core API patterns in this package against the official
MoveIt 2 Humble documentation and moveit2_tutorials source rather than
relying on general training-data recall, since moveit_py's Python bindings
have shifted meaningfully across MoveIt releases and older tutorials
(especially anything using `moveit_commander`) are silently wrong for
Humble. A few things I could **not** fully verify and flagged inline in the
code instead of guessing:

- **`ex5_planner_comparison.py`**: the exact method for extracting a plain
  `moveit_msgs/RobotTrajectory` from a `moveit_py` `RobotTrajectory` object
  (tries `get_robot_trajectory_msg()` then `to_msg()`, raises clearly if
  neither exists on your build).
- **`ex6_pilz_linear.py`**: CIRC planning needs a path constraint
  (center/interim point) that isn't exposed through the same
  `set_goal_state()` convenience path as PTP/LIN -- left as a documented
  gap rather than a guessed API call.
- **`ex7_orientation_constraints.py`**: whether `set_goal_state()` accepts
  `pose_stamped_msg` and `motion_plan_constraints` together (the official
  tutorial only demonstrates them separately). Also, true whole-trajectory
  path constraints (`PlanningComponent.set_path_constraints()`) are
  confirmed on the older `moveit_commander` API but not confirmed on
  `moveit_py` -- what's implemented here is a *goal* orientation constraint
  instead, with the gap noted in the script.
- **`ex8_attach_detach_object.py`**: the exact `PlanningSceneMonitor`
  read-write-context method for applying an `AttachedCollisionObject`
  (guessed as `process_attached_collision_object()` by analogy with the
  confirmed C++ `PlanningScene::processAttachedCollisionObjectMsg`, but not
  independently verified for the Python binding).

If any of these throw `AttributeError` on your install, that's expected —
each one has a comment immediately above it explaining what to check
(`help(...)` on the relevant class) and what the fallback approach is. This
is a reasonable place to spend real debugging time, since figuring out
which methods your specific `moveit_py` build actually exposes is itself
part of learning the API surface.

## 7. Full directory structure

The workspace tree for this project (top-level) is shown below for
reference.

```
ur5e_moveit_ws/
├── build/
│   ├── COLCON_IGNORE
│   ├── ur5e_motion_planning_lab/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── colcon_build.rc
   │   ├── colcon_command_prefix_setup_py.sh
│   │   ├── config/
│   │   │   └── moveit_py_params.yaml
│   │   ├── launch/
│   │   │   └── ur5e_fake_moveit.launch.py
│   │   ├── ur5e_motion_planning_lab/
│   │   │   ├── __init__.py
│   │   │   ├── ex1_joint_goal.py
│   │   │   ├── ex2_pose_goal.py
│   │   │   ├── ex3_cartesian_path.py
│   │   │   ├── ex4_collision_objects.py
│   │   │   ├── ex5_planner_comparison.py
│   │   │   ├── ex6_pilz_linear.py
│   │   │   ├── ex7_orientation_constraints.py
│   │   │   ├── ex8_attach_detach_object.py
│   │   │   └── mgi_base.py
│   └── prefix_override/
│       └── sitecustomize.py

├── install/
│   ├── setup.bash
│   ├── local_setup.bash
│   ├── setup.sh
│   ├── local_setup.sh
│   ├── setup.zsh
│   ├── local_setup.zsh
│   ├── setup.ps1
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── COLCON_IGNORE
+│   └── ur5e_motion_planning_lab/
│       ├── bin/
│       │   ├── ex1_joint_goal
│       │   ├── ex2_pose_goal
│       │   ├── ex3_cartesian_path
│       │   ├── ex4_collision_objects
│       │   ├── ex5_planner_comparison
│       │   ├── ex6_pilz_linear
│       │   ├── ex7_orientation_constraints
│       │   └── ex8_attach_detach_object
│       ├── lib/
│       └── share/
│           └── ur5e_motion_planning_lab/

├── log/
│   ├── latest/
│   │   └── ur5e_motion_planning_lab/
│   │       ├── streams.log
│   │       ├── command.log
│   │       ├── stdout.log
│   │       └── stderr.log
│   ├── latest_build/
│   └── build_*/

├── src/
│   └── ur5e_motion_planning_lab/
│       ├── package.xml
│       ├── setup.py
│       ├── README.md
│       ├── config/
│       │   └── moveit_py_params.yaml
│       ├── launch/
│       │   └── ur5e_fake_moveit.launch.py
│       ├── resource/
│       │   └── ur5e_motion_planning_lab
│       └── ur5e_motion_planning_lab/
│           ├── __init__.py
│           ├── ex1_joint_goal.py
│           ├── ex2_pose_goal.py
│           ├── ex3_cartesian_path.py
│           ├── ex4_collision_objects.py
│           ├── ex5_planner_comparison.py
│           ├── ex6_pilz_linear.py
│           ├── ex7_orientation_constraints.py
│           ├── ex8_attach_detach_object.py
│           └── mgi_base.py
```

Notes:
- The `src/ur5e_motion_planning_lab/` directory is the package you edit.
- `build/` and `install/` are generated by `colcon build` and can be
  safely removed and regenerated.
- `install/ur5e_motion_planning_lab/bin/` contains the runnable
  console scripts produced by the build.

