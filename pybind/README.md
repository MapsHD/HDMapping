# Python Bindings for HDMapping

Here, Python bindings for the general registration process are available. To compile the bindings when building the project, select `PYBIND` option in the CMake project. This will create the bindings for step 1 and step 2 of the registration process.

**Make sure to set the same Python interpreter that you plan to use in your project where you require bindings.**

The bindings can be freely imported to Python after compilation and used to speed up the processing once the suitable parameters for the given dataset are known and GUI is not anymore required for the experimentation. See usage example below (same as `example.py`). Lidar odometry:

```
import Release.lidar_odometry_py as lo
from pathlib import Path

# Fill the input directory and desired working directory
DATA_DIR = ...
WORKING_DIR = ...

import json
import sys
import lidar_odometry_py as lo

lo_params = lo.LidarOdometryParams()
lo_params.save_laz = True
lo_params.save_poses = True
lo_params.save_trajectory = True
lo_params.apply_consistency = False
lo_params.decimation = 0.01
lo_params.in_out_params_indoor.resolution_X = 0.3
lo_params.in_out_params_indoor.resolution_Y = 0.3
lo_params.in_out_params_indoor.resolution_Z = 0.3
lo_params.in_out_params_outdoor.resolution_X = 0.5
lo_params.in_out_params_outdoor.resolution_Y = 0.5
lo_params.in_out_params_outdoor.resolution_Z = 0.5
lo_params.filter_threshold_xy_inner = 2.0
lo_params.filter_threshold_xy_outer = 70.0
lo_params.threshould_output_filter = 2.0
lo_params.distance_bucket = 0.2
lo_params.polar_angle_deg = 10.0
lo_params.azimutal_angle_deg = 10.0
lo_params.robust_and_accurate_lidar_odometry_iterations = 20
lo_params.max_distance_lidar = 30.0
lo_params.use_robust_and_accurate_lidar_odometry = False
lo_params.nr_iter = 200
lo_params.fusionConventionNed = True
lo_params.fusionConventionNwu = False
lo_params.fusionConventionEnu = False
lo_params.threshold_nr_poses = 20
lo_params.num_constistency_iter = 10
lo.run_lidar_odometry(DATA_DIR, lo_params, WORKING_DIR, "all_lo.laz", "traj_lo.laz", "poses_lo.reg")
```

Multi-view registration:

```
import Release.multi_view_tls_registration_py as mvr

# Fill session path (from step 1) and desired output dir
SESSIO_PATH = ... 
OUTPUT_DIR = ...

mvr_params = mvr.TLSRegistration()
mvr_params.initial_pose_to_identity = True
mvr_params.save_laz = True
mvr_params.save_poses = True
mvr_params.save_trajectories_csv = True
mvr.run_multi_view_tls_registration(
    SESSION_PATH, 
    mvr_params, 
    OUTPUT_DIR,
)
```

To run multi-view registration with the different dataset formats:

* for ETH (no `traj_mvr.csv` as timestamps are not available in the data - adding `mvr_params.trajectories_name` will result in empty `traj_mvr.csv`): 
```
mvr.run_multi_view_tls_registration(
    str(DATA_DIR / "pairs.txt"),
    mvr_params,
)
```

* for RESSO (output will be a RESSO reg file as well, without sesion json; no `traj_mvr.csv` as timestamps are not available in the data - adding `mvr_params.trajectories_name` will result in empty `traj_mvr.csv`):
```
mvr.run_multi_view_tls_registration(
    str(DATA_DIR / "transformation_GroundTruth.reg"),
    mvr_params,
    str(DATA_DIR / "output.reg"),
)
```

* for 3DTK (data directory should contain txt files and their corresponding dat files) and WHU_TLS (data directory should contain las files):
```
mvr.run_multi_view_tls_registration(
    str(DATA_DIR),
    mvr_params,
)
```