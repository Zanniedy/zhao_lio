# Dynablox ROS2 / MID70 replay

Compiles the official ETH ASL Dynablox detection core and its Voxblox fork.
See `SOURCE.md` for exact revisions, retained licenses and port differences.

Build from the project root:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log_preprocessing build --base-paths src --packages-up-to dynablox_ros2 \
  --build-base build_preprocessing --install-base install_preprocessing --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 \
    -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt
source install_preprocessing/local_setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184 ros2 launch dynablox_ros2 replay.launch.py bag:=/path/to/rosbag2
```

Once built, `bash scripts/replay_preprocessed.sh /path/to/rosbag2` sets the local
environment, records its PID and prevents duplicate launches via a lock file.
See `docs/preprocessing.md` for the migrated workflow. No robot-workspace overlay
is required. `map2d:=false` disables the optional height projection viewer.

The replay disables the old input dynamic filter for a clean comparison, but
retains basic input filtering and existing LIO calibration. No hardware input,
navigation controller or motion publisher is started. Stop with Ctrl+C; do not
launch duplicate replays in the same domain. The 8-minute bag does not loop.

RViz: green is the current retained scan; red is official object-level dynamic
detection; yellow is unclassified/out-of-range input (not confirmed static).
Enable display 4 for ever-free seed points, 5 for historical ever-free voxels,
6 for the unfiltered world scan, 7 for the original LIO map, and 8 for the
non-dynamic TSDF surface. Historical ever-free does not mean currently empty.
The TSDF surface is a derived voxel-center visualization, not original PCD points.

`/dynablox/{kept,dynamic,pending}` preserve input frame, header and complete point
records. `/dynablox/*_world` and `/dynablox/static_map` are generated XYZ views in
`lio_odom`. `/dynablox/status` exposes counts, blocks, timing, queue latency and
bypasses. No raw input or LIO map is overwritten, and the TSDF surface is not a
saved navigation map. Algorithm startup, occlusion, sparse beams, pose errors and
stationary people remain limitations to inspect on the real bag.

## Historical validation in the robot workspace on 2026-11-17

Six C++ cases pass, covering official ever-free burn-in and unknown neighbors,
occupation revocation, seed-to-neighbor clustering, TSDF history/time reversal,
invalid inputs and new-object detection in previously free volume. Two ROS2 cases
verify organized-cloud row padding, raw timestamps/intensity/extra labels, invalid
point timing and missing-TF bypass. Snapshotted source checksums pass.

A 10.03-second real-bag sample received 100 detector outputs, all with `reason=ok`.
Five matched input/output frames preserve the complete multiset of raw point
records and identical headers/fields across retained, dynamic and unclassified
partitions. Mean processing was 9.82 ms (maximum 15.11 ms), with an additional
mean TF/queue wait of 171.19 ms (maximum 199.21 ms). Ten startup bypasses occurred
while LIO initialized; none were added during the sampled steady-state window.
These are local CPU measurements, not X5 acceptance or detection accuracy.

The source dataset is not person-labeled. A large red region may be a true moving
object or a false positive caused by pose error, sparsity or reflective surfaces.
The original LIO map is intentionally unchanged and can still contain people;
compare the current scan partitions and optional Voxblox surface separately.
