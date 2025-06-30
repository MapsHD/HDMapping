# Python Bindings for HDMapping

Here, Python bindings for the general registration process are available. To compile the bindings when building the project, select `PYBIND` option in the CMake project. This will create the bindings for step 1 and step 2 of the registration process.

**Make sure to set the same Python interpreter that you plan to use in your project where you require bindings.**

The bindings can be freely imported to Python after compilation and used to speed up the processing once the suitable parameters for the given dataset are known and GUI is not anymore required for the experimentation. See usage example below (same as `example.py`). Lidar odometry:

```
import os 
import Release.lidar_odometry_py as lo
from pathlib import Path

# Fill the input directory and desired working directory
DATA_DIR = ...
WORKING_DIR = ...
OUTPUT_DIR = ...

dummy_session = lo.Session()

lo_params = lo.LidarOdometryParams()
lo_params.decimation = 0.01
lo_params.in_out_params_indoor.resolution_X = 0.1
lo_params.in_out_params_indoor.resolution_Y = 0.1
lo_params.in_out_params_indoor.resolution_Z = 0.1
lo_params.in_out_params_outdoor.resolution_X = 0.3
lo_params.in_out_params_outdoor.resolution_Y = 0.3
lo_params.in_out_params_outdoor.resolution_Z = 0.3
lo_params.filter_threshold_xy_inner = 0.3
lo_params.filter_threshold_xy_outer = 70.0
lo_params.nr_iter = 100  
lo_params.sliding_window_trajectory_length_threshold = 200
lo_params.distance_bucket = 0.2
lo_params.polar_angle_deg = 10.0
lo_params.azimutal_angle_deg = 10.0
lo_params.robust_and_accurate_lidar_odometry_iterations = 20
lo_params.max_distance_lidar = 30.0
lo_params.use_robust_and_accurate_lidar_odometry = False
lo_params.fusionConventionNed = True
lo_params.fusionConventionNwu = False
lo_params.fusionConventionEnu = False
lo_params.apply_consistency = False
lo_params.threshold_nr_poses = 20
lo_params.num_constistency_iter = 10   
lo_params.threshould_output_filter = 0.3 
lo_params.min_counter = 10 

worker_data = lo.run_lidar_odometry(DATA_DIR, lo_params)
current_output_dir = lo.save_results_automatic(
    lo_params, worker_data, WORKING_DIR, 0.0
)
lo.save_all_to_las(
    worker_data=worker_data,
    params=lo_params,
    output_file_name=os.path.join(OUTPUT_DIR, "all_lo.laz"),
    session=dummy_session,
    export_selected=False,
    filter_on_export=True,
    apply_pose=True,
    add_to_pc_container=False,
)
lo.run_consistency(worker_data, lo_params)
current_output_dir = lo.save_results_automatic(
    lo_params, worker_data, WORKING_DIR, 0.0
)
lo.save_all_to_las(
    worker_data=worker_data,
    params=lo_params,
    output_file_name=os.path.join(OUTPUT_DIR, "all_consistency.laz"),
    session=dummy_session,
    export_selected=False,
    filter_on_export=True,
    apply_pose=True,
    add_to_pc_container=False,
)
```

Multi-view registration:

```
import os
import Release.lidar_odometry_py as lo
import Release.multi_view_tls_registration_py as mvr

# Fill session path (from step 1) and desired output dir
SESSION_PATH = ... 
OUTPUT_DIR = ...

mvr_params = mvr.TLSRegistration()
mvr_params.fuse_gnss = True
mvr_params.automatic_loop_closure = True
session = mvr.run_multi_view_tls_registration(
    self.input_session, mvr_params, self.gnss_dir
)
mvr.save_trajectories(
    session, 
    os.path.join(OUTPUT_DIR, "traj.csv"), 
    mvr_params.curve_consecutive_distance_meters, 
    mvr_params.not_curve_consecutive_distance_meters, 
    mvr_params.is_trajectory_export_downsampling
)
session.save_poses(os.path.join(WORKING_DIR, "poses.reg"))
session.save_session(os.path.join(WORKING_DIR, "session_final.json"))
mvr.save_all_to_las(session, os.path.join(OUTPUT_DIR, "all_final.las"))
```
