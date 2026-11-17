# Source and port boundary

- Dynablox: user-specified `/home/lsy/cs_project/open_source/dynablox`, commit
  `44a1537a7cac953a503cde13e81ebd86de3d3ebe`, ETH ASL, BSD-3-Clause.
- Voxblox: official `ethz-asl/voxblox`, branch `dynablox/release`, commit
  `32f43265780b85b2ed95e3793085a81ce764f708`, BSD-3-Clause.
- Minkindr: local dependency snapshot, commit
  `564f12639a8447d4d3e5e7707851424302941056`, retained upstream license.

The local Dynablox repository is read-only. `third_party` contains upstream
snapshots, not rewritten algorithm lookalikes. `SOURCE.sha256` lists their hashes.
The Minkindr README has only its final newline normalized by the patch tool.
The official clustering.cpp, tracking.cpp, ever_free_integrator.cpp and Voxblox
integration implementation are compiled directly, without algorithm edits.

`compat/dynablox` overlays four headers, removing unused ROS1 PCL includes and selecting
the ROS-independent configuration base in `include/dynablox_ros2/config.hpp`.
That base retains the upstream virtual parameter checks; ROS2 loads parameters
explicitly instead of ROS1 XmlRpc reflection. Original headers remain archived.
`compat/voxblox` adds explicit integral Eigen matrix indices in the projective
interpolator to compile with Eigen 3.4. That integrator is not selected for MID70;
the upstream factory nevertheless instantiates it. No interpolation rule changes.

`src/adapter.cpp` ports MotionDetector's indexing and orchestration:
index existing TSDF blocks -> official clustering -> official tracking ->
official ever-free update -> official TSDF integration. The serial point-map
construction has the same per-voxel updates as the upstream block-parallel loop.

MID70 uses the official Voxblox `merged` ray integrator, not the default dataset
configuration's `projective` integrator with a 2048x64 spinning-LiDAR model.
This is an explicit sensor/integration configuration difference. The default
Dynablox detector YAML thresholds are retained, including 26-neighbor ever-free
validation, burn-in 5, occupancy reset 150, and 20-point minimum clusters.

Additional ROS2 adapter safeguards: bounded TF queue, invalid-data/pose-jump
bypass, reset on time reversal/gap, rolling TSDF block retention and block budget.
Per-point LIO TF interpolation deskews a matching copy into the end-of-scan sensor
frame. Upstream integration then uses its single end-of-scan sensor origin.
Original raw records and timestamps are copied intact into split output clouds.
These outputs are not fed back into LIO: the detector depends on its poses.
Out-of-range returns are still passed to the official integrator so its
`allow_clear` behavior can observe free space up to the range limit.

No ROS1 evaluation/data download tools, ground-truth labels, semantic person
classifier, extra hand-written occupancy filter or custom cluster thresholds are
silently substituted for the official detector. This is a ROS2/MID70 port, not an
unmodified reproduction of the authors' dataset experiments.
