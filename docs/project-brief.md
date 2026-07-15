# Project brief

This is the living project brief for `auto-drone`. It captures the agreed
direction, verifies the starting point, and records decisions as work proceeds.
Update it whenever a goal, constraint, or validated project fact changes.

## Project objective

`auto-drone` is a portfolio-quality autonomy demo, with a future path to
real-world adaptation. Its near-term purpose is a lightweight, Mac-local ROS 2
simulation in which a drone maps and explores the Gazebo forest quickly enough
to show convincing autonomous behavior and a tree occupancy reconstruction.

The demonstration concerns trees and their coverage. The ground is not a
reconstruction target and should not consume meaningful mapping, planning, or
visualization work.

The active implementation goal is: **within a bounded forest volume, reach
100% of the fixed forest metric (all 12 trees and all eight occupied height
bands per tree) through safe, adaptive viewpoints.** The planner must re-plan
when coverage stops improving rather than remain at a single unproductive
viewpoint. This is a simulator metric and target, not a real-world guarantee.

### Exploration boundary

The planner uses an ENU software fence inside the map: X -21 to 20.5 m, Y -13
to 36.5 m, and Z 2 to 10.5 m. It rejects a target outside that fence and a
trajectory that comes within 2 m of conservative occupied evidence. Unknown
space is allowed so the drone can reach low-coverage regions; the active
segment is revalidated as new voxels arrive. This is a simulator planning
boundary, not a PX4 geofence.

## Architecture decision

The active working-tree implementation is a ROS 2 stack:

```text
Gazebo Harmonic depth camera + PX4 SITL
        -> ROS 2 tree occupancy map and coverage planner
        -> PX4 offboard velocity topics
        -> RViz occupancy inspection
```

This is the target architecture. Direct MAVSDK/Foxglove tooling has been
removed rather than maintained as a second runtime path.

### Mac runtime decision

Use a native Apple-Silicon ROS 2 Jazzy environment installed separately through
RoboStack/Conda. This keeps the existing Homebrew Gazebo Harmonic and PX4 SITL
installation native, avoiding the added graphics and networking cost of running
the simulator in a Linux VM or Docker Desktop. RViz is the initial ROS-native
occupancy viewer; it receives capped, tree-focused markers at a deliberately
low publish rate.

The RoboStack Jazzy environment supplies a matching prebuilt `ros_gz_bridge`,
so the project uses that standard bridge rather than maintaining a custom
Gazebo-depth adapter or compiling a second bridge copy locally.

## Verified starting point

Status checked on 2026-07-09 from the current working tree:

- The active Python core and ROS 2 nodes now share the single `auto_drone/`
  package; there is no second runtime source root.
- The current ROS entry point is `auto_drone.px4_control_node`; it maps depth,
  publishes tree occupancy, and runs the bounded coverage planner when enabled.
- `python3 -m compileall -q auto_drone` passes.
- Focused mapping and forest-metric tests run in the Conda ROS environment with
  unrelated pytest plugin autoloading disabled.
- The project relies on a local ignored PX4 checkout at
  `vendor/PX4-Autopilot`; the PX4 forest world and `x500_depth` model are not
  versioned here.

For detailed runtime values and integration assumptions, see
[current-state.md](current-state.md).

## Agreed demo outcome

Within a short, repeatable run in the existing forest world, the simulated
drone should autonomously explore and reconstruct trees with visibly high
coverage. The preferred runtime target is **30 seconds or less**.

The current simulator-only metric measures the number of fixed forest trees
observed and occupied height-band coverage around each reference tree.

## Priorities

1. **Runnable low-overhead ROS 2 workflow** — restore the ROS 2 simulator
   stack and remove or defer work that causes repeated Mac freezes.
2. **Tree-focused mapping and planning** — exclude or strongly de-prioritize
   ground data, then direct exploration toward useful tree observations.
3. **Fast, high-coverage demonstration** — tune and validate the system to
   produce the intended tree reconstruction in 30 seconds or less.
4. **Inspectable and defensible results** — use visualization and focused tests
   to make coverage, system state, and changed behavior easy to verify.

## Measurements

| Date | Measurement | Result | Consequence |
| --- | --- | --- | --- |
| 2026-07-09 | Current mapper, synthetic 640 x 480 depth frame, stride 10, 48 x 48 x 16 grid | 35.8 ms per mapped frame (average of 10) | Do not map at the 20 Hz control rate. The ROS 2 node will decouple low-cost control from a lower mapping rate, planning rate, and visualization rate. |
| 2026-07-09 | Current mapper at wider sampling strides | 13.3 ms/frame at stride 16; 8.6 ms/frame at stride 20 | Begin forest testing at stride 16; use stride 20 only if the visual coverage remains sufficient. |
| 2026-07-09 | ROS 2 baseline package | Colcon build succeeded; a second process received its status topic | Native Jazzy package discovery and DDS communication work on this Mac. |
| 2026-07-09 | PX4 v1.18 ROS message interface | Matching `px4_msgs` generated classes build and import successfully | ROS can now type-check PX4 odometry and offboard command messages. |
| 2026-07-09 | uXRCE-DDS Agent 2.4.3 build | Builds against the RoboStack Fast DDS/Fast CDR libraries with its macOS super-build disabled | Use the documented direct build; UDP is the only transport needed for SITL. |
| 2026-07-09 | PX4-to-ROS live check | PX4 forest SITL produced typed `/fmu` odometry and offboard topics through the local agent | The native ROS 2/PX4 control boundary is runnable on this Mac. |
| 2026-07-09 | Lean ROS controller live check | Controller received PX4 odometry and published 326 stationary velocity setpoints | The ROS controller can consume PX4 state and publish compatible setpoints without MAVSDK. |
| 2026-07-09 | ROS-Gazebo bridge | RoboStack `ros_gz_bridge` 1.0.22 starts in the Jazzy environment | Use the prebuilt bridge; its local source checkout expected interface 1.0.23. |
| 2026-07-09 | Live depth transport | Gazebo `/depth_camera` and `/camera_info` arrive in ROS as a 640 x 480 `32FC1` image and matching camera calibration | The application can now subscribe to real simulator depth without a Python Gazebo loop. |
| 2026-07-09 | Live mapping baseline | Forest depth integrated at stride 16; one measured pass took 37.6 ms | Map at most 5 Hz, keep control at 20 Hz, and publish a capped occupancy view at 1 Hz. |
| 2026-07-09 | Depth transport rate | Gazebo and ROS bridge sustain about 30 Hz; the reliable one-frame subscriber processed 85 map frames in a short sweep | The latest-frame QoS removes the earlier severe frame loss without building a backlog. |
| 2026-07-09 | Short sweep result | PX4 remained armed and in Offboard; the map reached 427 occupied voxels | The first altitude waypoint did not complete reliably, so this is not yet evidence of high tree coverage. |
| 2026-07-09 | Corrected sweep result | With unused position/acceleration fields set to `NaN`, PX4 completed takeoff, advanced the route, and integrated 122 depth frames into 1,381 occupied voxels | The second waypoint crossed a tree row and stalled; the active route now uses an elevated perimeter instead. |
| 2026-07-09 | Elevated short sweep | In about 30 seconds, PX4 stayed armed and in Offboard, reached waypoint 4 of 5, mapped 82 frames, and reached 5.63% known-map ratio | The route clears the prior collision; add tree-instance and per-tree-quality metrics next. |
| 2026-07-09 | Coverage metric foundation | Status now scores the 30 fixed forest tree references by observations and occupied height bands | This is a transparent simulator-only proxy for tree count and reconstruction quality. |
| 2026-07-09 | Timed tree coverage | In roughly 30 seconds: 17/30 trees observed (56.67%), 30.83% total height-band coverage, and 54.41% mean observed-tree quality | The demo has a repeatable metric baseline; improve route geometry and mapping settings to raise coverage. |
| 2026-07-09 | Learned planner check | The live planner selected `[9.38, -4.76, 8.0]` for a lowest-quality tree after map-clearance and fence checks | Fixed route progression is replaced with metric-driven target selection. |
| 2026-07-09 | Timed learned-planner run | After at least 60 seconds: 10/30 trees observed (33.33%), 16.67% total height-band coverage, 2.247% known-map ratio, and 404 occupied voxels | The learned planner is live and collision/fence constrained, but currently exhausts its candidates and underperforms the fixed-sweep baseline. |
| 2026-07-09 | RViz frame smoke test | A 20-second run published `map` to `occupancy_map` on `/tf_static` and occupancy markers in `occupancy_map` | The committed RViz configuration has a resolvable fixed frame and marker frame. |
| 2026-07-09 | Adaptive planner live check | Controller selected 9 m viewpoints and changed target while observations increased from 16 to 27 trees | Multi-height selection and re-planning are live; run a timed safety and full-coverage evaluation next. |
| 2026-07-09 | Adaptive coverage checkpoint | After an extended clean run: 30/30 trees observed, 42.5% height-band reconstruction, active 3 m target, and five excluded stagnant targets | Multi-height/no-repeat behavior works, but route efficiency and remaining vertical bands still prevent the 100% goal. |
| 2026-07-09 | Low-altitude inspection check | At a 3 m target, a fresh map reached 31.25% reconstruction in 13 frames; three trees reached all eight bands | Removing the over-broad takeoff guard materially improves vertical reconstruction. |

## Explicit scope guard

Until we agree otherwise, work targets the existing PX4 SITL and Gazebo forest
world on macOS. Real-hardware readiness, multi-drone support, additional
environments, and a broad configuration system are out of scope. ROS 2 is in
scope and is the intended integration architecture.

## Proposed delivery sequence

| Priority | Deliverable | Completion evidence |
| --- | --- | --- |
| P0 | Runnable ROS 2 baseline | A tracked bring-up path starts the forest simulation and required ROS 2 topics without the current iteration freezes. |
| P1 | Compute reduction | Profiling identifies expensive non-tree work; the active loop has explicit rate and data-volume limits appropriate for the Mac. |
| P2 | Tree-focused exploration | Mapping and planning exclude/de-prioritize ground and visibly target tree observations. |
| P3 | 30-second coverage demo | A timed run produces a defined, visually inspectable level of tree coverage within 30 seconds. |
| P4 | Regression coverage | Tests are collected by pytest and cover the behavior changed in P0–P3. |

P0 is the immediate implementation priority. P1 should be measured before
optimization so we remove the actual source of the Mac freezes rather than
guessing.

## Decision and progress log

| Date | Type | Entry | Evidence / follow-up |
| --- | --- | --- | --- |
| 2026-07-09 | Baseline | Current implementation uses ROS 2 for PX4 control and Gazebo bridging. | `auto_drone/`, `README.md`, and `docs/current-state.md` |
| 2026-07-09 | Decision | Use ROS 2 as the simulator-facing autonomy stack. | The direct Python/MAVSDK loop was removed. |
| 2026-07-09 | Decision | Optimize for a 30-second, tree-focused forest reconstruction demo on a Mac. | Ground reconstruction is not a goal; profile before reducing compute work. |
| 2026-07-09 | Decision | Use native RoboStack ROS 2 Jazzy and RViz first on Apple Silicon. | Avoid a Linux VM/Docker simulator; retain native Homebrew Gazebo/PX4. |
| 2026-07-09 | Measurement | Current depth mapping costs 35.8 ms/frame before simulator and visualization work. | Rate separation is required to keep the Mac responsive. |
| 2026-07-09 | Decision | Begin tree-mapping tests with depth stride 16. | It materially reduces mapper cost while retaining denser samples than stride 20. |
| 2026-07-09 | Decision | Use a normal Colcon install, not `--symlink-install`, in the Conda environment. | The current editable-install path rejects Colcon's option; normal installs build cleanly. |
| 2026-07-09 | Decision | Pin the local PX4 ROS interface to `px4_msgs` release/1.18 and Micro XRCE-DDS Agent 2.4.3. | This matches PX4 v1.18's Jazzy-compatible uXRCE-DDS contract. |
| 2026-07-09 | Decision | Build the agent directly against the Fast DDS libraries in the RoboStack Jazzy environment and disable SocketCAN. | This avoids the broken macOS super-build; UDP is the only transport required for SITL. |
| 2026-07-09 | Decision | Consolidate all runtime modules under `auto_drone/`. | A single import/package root keeps ROS installs and direct tools consistent. |
| 2026-07-09 | Decision | Use RoboStack's prebuilt `ros_gz_bridge` rather than a local bridge checkout. | It matches the installed interface package and removes an unnecessary local dependency. |
| 2026-07-09 | Decision | Keep mapping separate from the 20 Hz PX4 setpoint timer and leave the controller in stationary-hold mode during input validation. | This bounds compute cost and prevents unvalidated mapping from changing flight behavior. |
| 2026-07-09 | Decision | Ignore occupied depth hits below 0.5 m and display only occupied voxels above that threshold. | The demo measures tree reconstruction, not ground reconstruction. |
| 2026-07-09 | Superseded | The interim fixed sweep established a repeatable baseline. | It has been replaced by the bounded low-coverage planner. |
| 2026-07-09 | Constraint | Forest SITL accepts offboard mode but reports a failed preflight check when arming. | Keep the simulator-only `force_arm` option explicit and default-off; do not treat it as hardware behavior. |
| 2026-07-09 | Decision | Set unused PX4 trajectory fields to `NaN` in velocity control mode. | Zero position fields competed with velocity setpoints and prevented reliable takeoff. |
| 2026-07-09 | Cleanup | Use `auto-drone-ros` as the only Python environment and remove unused generic planner/controller files. | The active stack is smaller and its tests run with the documented Conda command. |
| 2026-07-09 | Decision | Use a bounded, collision-aware low-coverage planner. | It selects low-quality tree viewing points and rejects in-fence paths that violate mapped obstacle clearance. |
| 2026-07-09 | Decision | Permit unknown trajectory cells while rejecting mapped obstacles plus a 1 m clearance envelope. | Exploration must be able to leave known space while still using the live map for collision avoidance. |
| 2026-07-09 | Decision | Commit a minimal RViz occupancy configuration. | One command now opens the tree-occupancy marker display with the correct `map` fixed frame. |
| 2026-07-09 | Fix | Publish a static `map` to `occupancy_map` identity transform and place RViz's fixed-frame setting under `Global Options`. | RViz no longer depends on an undeclared map frame. |
| 2026-07-09 | Decision | Use RViz's Orbit camera for the occupancy configuration. | The reconstruction remains in a stable map frame while the viewer can navigate around it. |
| 2026-07-09 | Fix | Explicitly save the RViz Orbit view and begin at 20 m. | The map opens close enough to inspect instead of using RViz's 100 m default. |
| 2026-07-09 | Fix | Register RViz's `MoveCamera` tool in the occupancy configuration. | Orbit settings only take effect when the camera tool receives pointer input. |
| 2026-07-09 | Goal | Reach 100% of the fixed forest tree-height-band metric without remaining at a stagnant viewpoint. | Add multi-height coverage viewpoints and re-plan when the live metric stops improving. |
| 2026-07-09 | Decision | Reject a direct flight path that intersects the fixed forest tree-reference clearance envelope. | The live map alone cannot rule out a first encounter with an unknown tree; this simulator-only guard prevents known-reference collisions. |
| 2026-07-09 | Decision | Keep stagnant viewpoint targets excluded until coverage improves. | A timeout must force a genuinely different inspection view, not repeatedly reselect the same location. |
| 2026-07-09 | Fix | Sort candidate viewpoints by missing-band height priority before travel distance. | The planner must not discard its vertical coverage intent in favor of the closest altitude. |
| 2026-07-09 | Fix | Trigger a stagnation re-plan only within 2 m of the viewpoint. | The first timeout implementation churned targets during transit rather than allowing an inspection to occur. |
| 2026-07-09 | Fix | Apply the 8 m takeoff guard only before selecting an exploration target. | A broad guard silently prevented all planned 3 m and 5 m inspections. |
| 2026-07-09 | Decision | Prefer the nearest safe viewpoint among trees within one missing-band level of the least-covered tree. | This reduces cross-map flights without abandoning materially useful coverage. |
| 2026-07-09 | Decision | Raise the RViz occupancy cap to 8,000 voxels. | The full forest reconstruction is more inspectable while retaining a bounded Mac workload. |
| 2026-07-09 | Decision | Replace the overlapping grass patches and 30-tree layout with a 12-tree grid and a local non-textured ground model. | The active benchmark, tree references, and planner clearance guard now share the simpler layout. |

## How this document will be maintained

For each substantive task, record only the information needed to keep the
project aligned:

1. Restate the goal and success check before changing code when the task is
   multi-step or ambiguous.
2. Add a decision-log entry when a goal, assumption, interface, dependency, or
   validation result changes.
3. Keep `current-state.md` factual: distinguish verified behavior from intended
   behavior and list blockers rather than presenting them as complete.
4. Do not expand scope silently; unresolved choices stay explicitly marked for
   confirmation.
