# Python Bindings for HDMapping

Here, Python bindings for the general registration process are available. To compile the bindings when building the project, select `PYBIND` option in the CMake project. This will create the bindings for step 1 and step 2 of the registration process.

The bindings can be freely imported to Python after compilation and used to speed up the processing once the suitable parameters for the given dataset are known and GUI is not anymore required for the experimentation. See usage example below (same as `example.py`):

```
import Release.lidar_odometry_py as lo
import Release.multi_view_tls_registration_py as mvr
from pathlib import Path

DATA_DIR = Path(R"A:\hdmapping\data")

lo_params = lo.LidarOdometryParams()
lo_params.in_out_params.resolution_X = 0.3
lo_params.in_out_params.resolution_Y = 0.3
lo_params.in_out_params.resolution_Z = 0.3

lo.run_lidar_odometry(
    str(DATA_DIR),
    lo_params,
    output_las_name=str(DATA_DIR / "all.laz"),
    trajectory_ascii_name=str(DATA_DIR / "traj_lo.csv"),
)

mvr_params = mvr.TLSRegistration()
mvr_params.initial_pose_to_identity = True
mvr_params.output_las_name = str(DATA_DIR / "all_mvr.laz")
mvr_params.trajectories_name = str(DATA_DIR / "traj_mvr.csv")
mvr.run_multi_view_tls_registration(
    str(DATA_DIR), str(DATA_DIR / "session.json"), mvr_params
)
```
