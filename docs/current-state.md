# Current project state

Status snapshot: 2026-07-09.

The active implementation is a single ROS 2 package, `auto_drone/`. The old
direct Python/MAVSDK runner, Foxglove publisher, Gazebo Python subscribers, and
trajectory probe were removed to avoid maintaining two autonomy architectures.

The active outdoor benchmark has 12 irregularly placed trees, a visitor center,
pavilion, footbridge, service vehicle, construction site, and boulder field on
a local matte ground model. It replaces the overlapping Fuel
grass patches and 30-tree layout; older 30-tree measurements below are retained
only as historical results and are not comparable to this benchmark.

## Verified baseline

- ROS 2 Jazzy runs in the local `auto-drone-ros` Conda environment.
- PX4 v1.18 `px4_msgs` builds in this workspace.
- The native Micro XRCE-DDS Agent connects PX4 SITL to ROS 2 over UDP 8888.
- PX4 odometry and offboard-setpoint topics are visible in ROS 2.
- Gazebo depth images and camera calibration are visible in ROS 2 through the
  standard `ros_gz_bridge`.
- `px4_control_node` receives PX4 odometry and publishes stationary velocity
  setpoints at 20 Hz without arming the vehicle; it now also reports receipt
  of depth and camera-calibration messages.

The node includes a bounded low-coverage planner. It is disabled by default;
only `enable_exploration:=true` requests PX4 offboard mode and arming. Mapping
and visualisation stay independent of the 20 Hz setpoint timer.

The current forest SITL starts with a failing preflight check. Its explicit
`force_arm:=true` option is only for this simulator validation and is not a
hardware-ready behavior.

The local PX4 forest instance also defaults to Return on a missing GCS link.
Set its local `NAV_DLL_ACT` parameter to `0` before a simulator sweep. With
that override, PX4 remains armed and in Offboard when unused trajectory fields
are `NaN`. The fixed route was removed: after takeoff, the active planner
derives free/unknown frontiers from the belief map, selects a bounded set of
informative viewpoints, and follows a coarse route with mapped-obstacle
clearance. Planning treats a single occupied depth observation as a safety
obstacle, keeps a 2 m envelope around it, and revalidates the active segment
after every newly integrated map frame. It has no tree locations or other
scene-specific inputs.

The former elevated sweep has been verified for a short run: PX4 remained armed and
in Offboard, reached route waypoint 4 of 5, mapped 82 frames, and reached a
5.63% known-map ratio. In that same roughly 30-second run, the forest metric
reported 17 of 30 trees observed (56.67%), 30.83% total height-band coverage,
and 54.41% mean quality for observed trees. This is the first repeatable
tree-focused coverage baseline; it still needs tuning for higher coverage.

The learned planner builds and has focused collision/boundary tests. Its first
live forest check selected `[9.38, -4.76, 8.0]`: a 7 m viewing point for a
lowest-quality tree after fence and mapped-clearance checks passed.

In a subsequent learned-planner run lasting at least one minute, PX4 stayed
armed and in Offboard without failsafe. The mapper processed 173 frames and
reported 404 occupied voxels, a 2.247% known-map ratio, 10 of 30 observed trees
(33.33%), and 16.67% height-band reconstruction. The planner eventually had no
eligible target. This validates the live data path and constraints, but it is
not yet a coverage improvement over the fixed-sweep baseline.

RViz now uses the `map` fixed frame while the node publishes a static identity
transform from `map` to `occupancy_map`; occupied markers use the child frame.
This was verified in a 20-second simulator run: both the transform and marker
frame were present, with 15 mapped frames and 2 of 30 trees observed. That
short run is a visualization smoke test, not a coverage benchmark.

The committed RViz setup uses an Orbit camera focused on the forest volume.
The map frame remains fixed; only the camera moves, so the occupied voxels can
be rotated, panned, and zoomed without changing the reconstruction coordinates.
It explicitly registers RViz's `MoveCamera` tool; the Orbit view settings alone
do not create a mouse-input tool.

The status stream reports map-generic progress: known-map ratio, occupied floor
ratio, frontier-cell count, selected route length, and planning time. The
legacy forest metric remains available only as an offline benchmark helper; it
is not used by the controller.

The current node integrates depth at a 90 ms cadence with stride 8 across the
camera's valid 0.25-19 m range, including floor hits, and publishes up to
16,000 occupied voxels on `/auto_drone_px4_control/occupied_voxels` four times
per second for RViz. Successive frames cycle through all 64 stride-8 pixel
phases, so throughput stays bounded without repeatedly sampling the same pixel
lattice. It holds a bounded 12-frame RGB and depth queue and a longer
ground-truth pose history, selecting only the newest exact timestamp pair that
also has a capture-time pose. The hot path decodes those images into NumPy
views, avoiding a full-frame Python object conversion before sampling rays.

RGB hit association is implemented with a bounded per-voxel running mean and
per-point RViz marker colors. The separate IMX214 RGB and StereoOV7251 depth
sensors share pose, intrinsics, 640x480 resolution, and a 10 Hz update rate, but
are fused only when their capture timestamps are identical. Their extra Gazebo
sensor previews are disabled to reduce render latency. Exploration is capped at
1 m/s with 1 m/s² acceleration.

Camera projection uses timestamped Gazebo ground-truth odometry for perception
and visualization while PX4 odometry remains the control input. Each accepted
RGB-depth pair robustly fits the known planar benchmark floor to recover the
camera payload's capture-time height, roll, and pitch; a frame that cannot be
registered is not integrated. The combined X500 mount and OakD optical-center
extrinsic is then applied before each pixel endpoint is voxelized once in world
space. Floor rays that do not intersect z=0 inside the mapping range are not
integrated; this prevents a shallow, out-of-range ground ray from creating an
elevated blue endpoint. Ground correction uses the same staggered pixel phase
as endpoint projection. There is no per-voxel height or color mask, and
occupancy must receive three consistent hits before publication. Separate 20
Hz marker streams carry the X500 mesh and camera frustum.

The active planner runs at most once per second. It bins known-free cells next
to unknown cells into frontier clusters, with extra gain for floor and occupied
surface boundaries. It evaluates the four highest-gain eligible clusters,
searching past recently completed viewpoints, and first uses a coarse A* route
through known-free cells. If no fully observed route exists, it permits unknown
navigation at a high cost while still blocking inflated conservative obstacle
evidence. Paths are simplified into a small waypoint queue and retained during
transit. Only reached endpoints are excluded to avoid doubling back; an
excluded target is not immediately re-enabled as a fallback. A retained
segment is discarded when new RGB-D evidence makes it unsafe. If that evidence
places the current pose inside the clearance envelope, the controller
backtracks along its recent safe-pose history before replanning.

In the corrected live validation run on 2026-07-15, the drone crossed multiple
tree rows without a PX4 failsafe, integrated 375 synchronized RGB-D frames,
reached a 22.33% known-volume ratio and 5,013 published occupied voxels,
completed two viewpoints, and recovered from three enclosed-pose events. No
ground-colored occupied voxel was reported above z=0. This validates continuous
mapping, non-repeating route replacement, floor projection, and conservative
collision recovery, but not yet full frontier completion.

The first live adaptive check selected a 9 m target, then changed from
`[20.81, 20.21, 9.0]` to `[-10.63, 5.05, 9.0]` while observed trees increased
from 16 to 27. It demonstrates target switching and multi-height selection;
it is not yet evidence of full height-band coverage or collision-free
completion.

In an extended clean run with the no-repeat rule active, all 30 tree references
were observed and the controller descended to a 3 m target after excluding five
stagnant views. Reconstruction reached 42.5% of height bands. This verifies
adaptive vertical target selection, but it is below the active 100% goal.

After the takeoff-guard correction, the vehicle reached a 3 m target and a
fresh map reached 31.25% reconstruction in 13 frames, including three trees
with all eight bands. Low-altitude inspection is now physically exercised.

## Performance baseline

The earlier mapper took about 34 ms per 640×480 frame at depth stride 10. It
took 13 ms at stride 16 and 9 ms at stride 20. The eventual autonomy node will
use independent control, mapping, planning, and visualization rates; mapping
will begin at stride 16 and must not run at the 20 Hz control rate.
